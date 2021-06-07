#include "cache.h"
#include "checkout.h"
#include "config.h"
#include "builtin.h"
#include "dir.h"
#include "parse-options.h"
#include "argv-array.h"
#include "branch.h"
#include "refs.h"
#include "run-command.h"
#include "sigchain.h"
#include "refs.h"
#include "utf8.h"
#include "worktree.h"

static const char * const worktree_usage[] = {
	N_("git worktree add [<options>] <path> [<commit-ish>]"),
	N_("git worktree list [<options>]"),
	N_("git worktree lock [<options>] <path>"),
	N_("git worktree move <worktree> <new-path>"),
	N_("git worktree prune [<options>]"),
	N_("git worktree remove [<options>] <worktree>"),
	N_("git worktree unlock <path>"),
	NULL
};

struct add_opts {
	int force;
	int detach;
	int checkout;
	int keep_locked;
	const char *new_branch;
	int force_new_branch;
};

static int show_only;
static int verbose;
static int guess_remote;
static timestamp_t expire;

static int git_worktree_config(const char *var, const char *value, void *cb)
{
	if (!strcmp(var, "worktree.guessremote")) {
		guess_remote = git_config_bool(var, value);
		return 0;
	}

	return git_default_config(var, value, cb);
}

static int prune_worktree(const char *id, struct strbuf *reason)
{
	struct stat st;
	char *path;
	int fd;
	size_t len;
	ssize_t read_result;

	if (!is_directory(git_path("worktrees/%s", id))) {
		strbuf_addf(reason, _("Removing worktrees/%s: not a valid directory"), id);
		return 1;
	}
	if (file_exists(git_path("worktrees/%s/locked", id)))
		return 0;
	if (stat(git_path("worktrees/%s/gitdir", id), &st)) {
		strbuf_addf(reason, _("Removing worktrees/%s: gitdir file does not exist"), id);
		return 1;
	}
	fd = open(git_path("worktrees/%s/gitdir", id), O_RDONLY);
	if (fd < 0) {
		strbuf_addf(reason, _("Removing worktrees/%s: unable to read gitdir file (%s)"),
			    id, strerror(errno));
		return 1;
	}
	len = xsize_t(st.st_size);
	path = xmallocz(len);

	read_result = read_in_full(fd, path, len);
	if (read_result < 0) {
		strbuf_addf(reason, _("Removing worktrees/%s: unable to read gitdir file (%s)"),
			    id, strerror(errno));
		close(fd);
		free(path);
		return 1;
	}
	close(fd);

	if (read_result != len) {
		strbuf_addf(reason,
			    _("Removing worktrees/%s: short read (expected %"PRIuMAX" bytes, read %"PRIuMAX")"),
			    id, (uintmax_t)len, (uintmax_t)read_result);
		free(path);
		return 1;
	}
	while (len && (path[len - 1] == '\n' || path[len - 1] == '\r'))
		len--;
	if (!len) {
		strbuf_addf(reason, _("Removing worktrees/%s: invalid gitdir file"), id);
		free(path);
		return 1;
	}
	path[len] = '\0';
	if (!file_exists(path)) {
		struct stat st_link;
		free(path);
		/*
		 * the repo is moved manually and has not been
		 * accessed since?
		 */
		if (!stat(git_path("worktrees/%s/link", id), &st_link) &&
		    st_link.st_nlink > 1)
			return 0;
		if (st.st_mtime <= expire) {
			strbuf_addf(reason, _("Removing worktrees/%s: gitdir file points to non-existent location"), id);
			return 1;
		} else {
			return 0;
		}
	}
	free(path);
	return 0;
}

static void prune_worktrees(void)
{
	struct strbuf reason = STRBUF_INIT;
	struct strbuf path = STRBUF_INIT;
	DIR *dir = opendir(git_path("worktrees"));
	struct dirent *d;
	int ret;
	if (!dir)
		return;
	while ((d = readdir(dir)) != NULL) {
		if (is_dot_or_dotdot(d->d_name))
			continue;
		strbuf_reset(&reason);
		if (!prune_worktree(d->d_name, &reason))
			continue;
		if (show_only || verbose)
			printf("%s\n", reason.buf);
		if (show_only)
			continue;
		git_path_buf(&path, "worktrees/%s", d->d_name);
		ret = remove_dir_recursively(&path, 0);
		if (ret < 0 && errno == ENOTDIR)
			ret = unlink(path.buf);
		if (ret)
			error_errno(_("failed to remove '%s'"), path.buf);
	}
	closedir(dir);
	if (!show_only)
		rmdir(git_path("worktrees"));
	strbuf_release(&reason);
	strbuf_release(&path);
}

static int prune(int ac, const char **av, const char *prefix)
{
	struct option options[] = {
		OPT__DRY_RUN(&show_only, N_("do not remove, show only")),
		OPT__VERBOSE(&verbose, N_("report pruned working trees")),
		OPT_EXPIRY_DATE(0, "expire", &expire,
				N_("expire working trees older than <time>")),
		OPT_END()
	};

	expire = TIME_MAX;
	ac = parse_options(ac, av, prefix, options, worktree_usage, 0);
	if (ac)
		usage_with_options(worktree_usage, options);
	prune_worktrees();
	return 0;
}

static char *junk_work_tree;
static char *junk_git_dir;
static int is_junk;
static pid_t junk_pid;

static void remove_junk(void)
{
	struct strbuf sb = STRBUF_INIT;
	if (!is_junk || getpid() != junk_pid)
		return;
	if (junk_git_dir) {
		strbuf_addstr(&sb, junk_git_dir);
		remove_dir_recursively(&sb, 0);
		strbuf_reset(&sb);
	}
	if (junk_work_tree) {
		strbuf_addstr(&sb, junk_work_tree);
		remove_dir_recursively(&sb, 0);
	}
	strbuf_release(&sb);
}

static void remove_junk_on_signal(int signo)
{
	remove_junk();
	sigchain_pop(signo);
	raise(signo);
}

static const char *worktree_basename(const char *path, int *olen)
{
	const char *name;
	int len;

	len = strlen(path);
	while (len && is_dir_sep(path[len - 1]))
		len--;

	for (name = path + len - 1; name > path; name--)
		if (is_dir_sep(*name)) {
			name++;
			break;
		}

	*olen = len;
	return name;
}

static int add_worktree(const char *path, const char *refname,
			const struct add_opts *opts)
{
	struct strbuf sb_git = STRBUF_INIT, sb_repo = STRBUF_INIT;
	struct strbuf sb = STRBUF_INIT;
	const char *name;
	struct stat st;
	struct child_process cp = CHILD_PROCESS_INIT;
	struct argv_array child_env = ARGV_ARRAY_INIT;
	int counter = 0, len, ret;
	struct strbuf symref = STRBUF_INIT;
	struct commit *commit = NULL;
	int is_branch = 0;

	if (file_exists(path) && !is_empty_dir(path))
		die(_("'%s' already exists"), path);

	/* is 'refname' a branch or commit? */
	if (!opts->detach && !strbuf_check_branch_ref(&symref, refname) &&
	    ref_exists(symref.buf)) {
		is_branch = 1;
		if (!opts->force)
			die_if_checked_out(symref.buf, 0);
	}
	commit = lookup_commit_reference_by_name(refname);
	if (!commit)
		die(_("invalid reference: %s"), refname);

	name = worktree_basename(path, &len);
	git_path_buf(&sb_repo, "worktrees/%.*s", (int)(path + len - name), name);
	len = sb_repo.len;
	if (safe_create_leading_directories_const(sb_repo.buf))
		die_errno(_("could not create leading directories of '%s'"),
			  sb_repo.buf);
	while (!stat(sb_repo.buf, &st)) {
		counter++;
		strbuf_setlen(&sb_repo, len);
		strbuf_addf(&sb_repo, "%d", counter);
	}
	name = strrchr(sb_repo.buf, '/') + 1;

	junk_pid = getpid();
	atexit(remove_junk);
	sigchain_push_common(remove_junk_on_signal);

	if (mkdir(sb_repo.buf, 0777))
		die_errno(_("could not create directory of '%s'"), sb_repo.buf);
	junk_git_dir = xstrdup(sb_repo.buf);
	is_junk = 1;

	/*
	 * lock the incomplete repo so prune won't delete it, unlock
	 * after the preparation is over.
	 */
	strbuf_addf(&sb, "%s/locked", sb_repo.buf);
	if (!opts->keep_locked)
		write_file(sb.buf, "initializing");
	else
		write_file(sb.buf, "added with --lock");

	strbuf_addf(&sb_git, "%s/.git", path);
	if (safe_create_leading_directories_const(sb_git.buf))
		die_errno(_("could not create leading directories of '%s'"),
			  sb_git.buf);
	junk_work_tree = xstrdup(path);

	strbuf_reset(&sb);
	strbuf_addf(&sb, "%s/gitdir", sb_repo.buf);
	write_file(sb.buf, "%s", real_path(sb_git.buf));
	write_file(sb_git.buf, "gitdir: %s/worktrees/%s",
		   real_path(get_git_common_dir()), name);
	/*
	 * This is to keep resolve_ref() happy. We need a valid HEAD
	 * or is_git_directory() will reject the directory. Any value which
	 * looks like an object ID will do since it will be immediately
	 * replaced by the symbolic-ref or update-ref invocation in the new
	 * worktree.
	 */
	strbuf_reset(&sb);
	strbuf_addf(&sb, "%s/HEAD", sb_repo.buf);
	write_file(sb.buf, "%s", sha1_to_hex(null_sha1));
	strbuf_reset(&sb);
	strbuf_addf(&sb, "%s/commondir", sb_repo.buf);
	write_file(sb.buf, "../..");

	fprintf_ln(stderr, _("Preparing %s (identifier %s)"), path, name);

	argv_array_pushf(&child_env, "%s=%s", GIT_DIR_ENVIRONMENT, sb_git.buf);
	argv_array_pushf(&child_env, "%s=%s", GIT_WORK_TREE_ENVIRONMENT, path);
	cp.git_cmd = 1;

	if (!is_branch)
		argv_array_pushl(&cp.args, "update-ref", "HEAD",
				 oid_to_hex(&commit->object.oid), NULL);
	else
		argv_array_pushl(&cp.args, "symbolic-ref", "HEAD",
				 symref.buf, NULL);
	cp.env = child_env.argv;
	ret = run_command(&cp);
	if (ret)
		goto done;

	if (opts->checkout) {
		cp.argv = NULL;
		argv_array_clear(&cp.args);
		argv_array_pushl(&cp.args, "reset", "--hard", NULL);
		cp.env = child_env.argv;
		ret = run_command(&cp);
		if (ret)
			goto done;
	}

	is_junk = 0;
	FREE_AND_NULL(junk_work_tree);
	FREE_AND_NULL(junk_git_dir);

done:
	if (ret || !opts->keep_locked) {
		strbuf_reset(&sb);
		strbuf_addf(&sb, "%s/locked", sb_repo.buf);
		unlink_or_warn(sb.buf);
	}

	/*
	 * Hook failure does not warrant worktree deletion, so run hook after
	 * is_junk is cleared, but do return appropriate code when hook fails.
	 */
	if (!ret && opts->checkout) {
		const char *hook = find_hook("post-checkout");
		if (hook) {
			const char *env[] = { "GIT_DIR", "GIT_WORK_TREE", NULL };
			cp.git_cmd = 0;
			cp.no_stdin = 1;
			cp.stdout_to_stderr = 1;
			cp.dir = path;
			cp.env = env;
			cp.argv = NULL;
			argv_array_pushl(&cp.args, absolute_path(hook),
					 oid_to_hex(&null_oid),
					 oid_to_hex(&commit->object.oid),
					 "1", NULL);
			ret = run_command(&cp);
		}
	}

	argv_array_clear(&child_env);
	strbuf_release(&sb);
	strbuf_release(&symref);
	strbuf_release(&sb_repo);
	strbuf_release(&sb_git);
	return ret;
}

static int add(int ac, const char **av, const char *prefix)
{
	struct add_opts opts;
	const char *new_branch_force = NULL;
	char *path;
	const char *branch;
	const char *opt_track = NULL;
	struct option options[] = {
		OPT__FORCE(&opts.force,
			   N_("checkout <branch> even if already checked out in other worktree"),
			   PARSE_OPT_NOCOMPLETE),
		OPT_STRING('b', NULL, &opts.new_branch, N_("branch"),
			   N_("create a new branch")),
		OPT_STRING('B', NULL, &new_branch_force, N_("branch"),
			   N_("create or reset a branch")),
		OPT_BOOL(0, "detach", &opts.detach, N_("detach HEAD at named commit")),
		OPT_BOOL(0, "checkout", &opts.checkout, N_("populate the new working tree")),
		OPT_BOOL(0, "lock", &opts.keep_locked, N_("keep the new working tree locked")),
		OPT_PASSTHRU(0, "track", &opt_track, NULL,
			     N_("set up tracking mode (see git-branch(1))"),
			     PARSE_OPT_NOARG | PARSE_OPT_OPTARG),
		OPT_BOOL(0, "guess-remote", &guess_remote,
			 N_("try to match the new branch name with a remote-tracking branch")),
		OPT_END()
	};

	memset(&opts, 0, sizeof(opts));
	opts.checkout = 1;
	ac = parse_options(ac, av, prefix, options, worktree_usage, 0);
	if (!!opts.detach + !!opts.new_branch + !!new_branch_force > 1)
		die(_("-b, -B, and --detach are mutually exclusive"));
	if (ac < 1 || ac > 2)
		usage_with_options(worktree_usage, options);

	path = prefix_filename(prefix, av[0]);
	branch = ac < 2 ? "HEAD" : av[1];

	if (!strcmp(branch, "-"))
		branch = "@{-1}";

	opts.force_new_branch = !!new_branch_force;
	if (opts.force_new_branch) {
		struct strbuf symref = STRBUF_INIT;

		opts.new_branch = new_branch_force;

		if (!opts.force &&
		    !strbuf_check_branch_ref(&symref, opts.new_branch) &&
		    ref_exists(symref.buf))
			die_if_checked_out(symref.buf, 0);
		strbuf_release(&symref);
	}

	if (ac < 2 && !opts.new_branch && !opts.detach) {
		int n;
		const char *s = worktree_basename(path, &n);
		opts.new_branch = xstrndup(s, n);
		if (guess_remote) {
			struct object_id oid;
			const char *remote =
				unique_tracking_name(opts.new_branch, &oid);
			if (remote)
				branch = remote;
		}
	}

	if (ac == 2 && !opts.new_branch && !opts.detach) {
		struct object_id oid;
		struct commit *commit;
		const char *remote;

		commit = lookup_commit_reference_by_name(branch);
		if (!commit) {
			remote = unique_tracking_name(branch, &oid);
			if (remote) {
				opts.new_branch = branch;
				branch = remote;
			}
		}
	}

	if (opts.new_branch) {
		struct child_process cp = CHILD_PROCESS_INIT;
		cp.git_cmd = 1;
		argv_array_push(&cp.args, "branch");
		if (opts.force_new_branch)
			argv_array_push(&cp.args, "--force");
		argv_array_push(&cp.args, opts.new_branch);
		argv_array_push(&cp.args, branch);
		if (opt_track)
			argv_array_push(&cp.args, opt_track);
		if (run_command(&cp))
			return -1;
		branch = opts.new_branch;
	} else if (opt_track) {
		die(_("--[no-]track can only be used if a new branch is created"));
	}

	UNLEAK(path);
	UNLEAK(opts);
	return add_worktree(path, branch, &opts);
}

static void show_worktree_porcelain(struct worktree *wt)
{
	printf("worktree %s\n", wt->path);
	if (wt->is_bare)
		printf("bare\n");
	else {
		printf("HEAD %s\n", oid_to_hex(&wt->head_oid));
		if (wt->is_detached)
			printf("detached\n");
		else if (wt->head_ref)
			printf("branch %s\n", wt->head_ref);
	}
	printf("\n");
}

static void show_worktree(struct worktree *wt, int path_maxlen, int abbrev_len)
{
	struct strbuf sb = STRBUF_INIT;
	int cur_path_len = strlen(wt->path);
	int path_adj = cur_path_len - utf8_strwidth(wt->path);

	strbuf_addf(&sb, "%-*s ", 1 + path_maxlen + path_adj, wt->path);
	if (wt->is_bare)
		strbuf_addstr(&sb, "(bare)");
	else {
		strbuf_addf(&sb, "%-*s ", abbrev_len,
				find_unique_abbrev(wt->head_oid.hash, DEFAULT_ABBREV));
		if (wt->is_detached)
			strbuf_addstr(&sb, "(detached HEAD)");
		else if (wt->head_ref) {
			char *ref = shorten_unambiguous_ref(wt->head_ref, 0);
			strbuf_addf(&sb, "[%s]", ref);
			free(ref);
		} else
			strbuf_addstr(&sb, "(error)");
	}
	printf("%s\n", sb.buf);

	strbuf_release(&sb);
}

static void measure_widths(struct worktree **wt, int *abbrev, int *maxlen)
{
	int i;

	for (i = 0; wt[i]; i++) {
		int sha1_len;
		int path_len = strlen(wt[i]->path);

		if (path_len > *maxlen)
			*maxlen = path_len;
		sha1_len = strlen(find_unique_abbrev(wt[i]->head_oid.hash, *abbrev));
		if (sha1_len > *abbrev)
			*abbrev = sha1_len;
	}
}

static int list(int ac, const char **av, const char *prefix)
{
	int porcelain = 0;

	struct option options[] = {
		OPT_BOOL(0, "porcelain", &porcelain, N_("machine-readable output")),
		OPT_END()
	};

	ac = parse_options(ac, av, prefix, options, worktree_usage, 0);
	if (ac)
		usage_with_options(worktree_usage, options);
	else {
		struct worktree **worktrees = get_worktrees(GWT_SORT_LINKED);
		int path_maxlen = 0, abbrev = DEFAULT_ABBREV, i;

		if (!porcelain)
			measure_widths(worktrees, &abbrev, &path_maxlen);

		for (i = 0; worktrees[i]; i++) {
			if (porcelain)
				show_worktree_porcelain(worktrees[i]);
			else
				show_worktree(worktrees[i], path_maxlen, abbrev);
		}
		free_worktrees(worktrees);
	}
	return 0;
}

static int lock_worktree(int ac, const char **av, const char *prefix)
{
	const char *reason = "", *old_reason;
	struct option options[] = {
		OPT_STRING(0, "reason", &reason, N_("string"),
			   N_("reason for locking")),
		OPT_END()
	};
	struct worktree **worktrees, *wt;

	ac = parse_options(ac, av, prefix, options, worktree_usage, 0);
	if (ac != 1)
		usage_with_options(worktree_usage, options);

	worktrees = get_worktrees(0);
	wt = find_worktree(worktrees, prefix, av[0]);
	if (!wt)
		die(_("'%s' is not a working tree"), av[0]);
	if (is_main_worktree(wt))
		die(_("The main working tree cannot be locked or unlocked"));

	old_reason = is_worktree_locked(wt);
	if (old_reason) {
		if (*old_reason)
			die(_("'%s' is already locked, reason: %s"),
			    av[0], old_reason);
		die(_("'%s' is already locked"), av[0]);
	}

	write_file(git_common_path("worktrees/%s/locked", wt->id),
		   "%s", reason);
	free_worktrees(worktrees);
	return 0;
}

static int unlock_worktree(int ac, const char **av, const char *prefix)
{
	struct option options[] = {
		OPT_END()
	};
	struct worktree **worktrees, *wt;
	int ret;

	ac = parse_options(ac, av, prefix, options, worktree_usage, 0);
	if (ac != 1)
		usage_with_options(worktree_usage, options);

	worktrees = get_worktrees(0);
	wt = find_worktree(worktrees, prefix, av[0]);
	if (!wt)
		die(_("'%s' is not a working tree"), av[0]);
	if (is_main_worktree(wt))
		die(_("The main working tree cannot be locked or unlocked"));
	if (!is_worktree_locked(wt))
		die(_("'%s' is not locked"), av[0]);
	ret = unlink_or_warn(git_common_path("worktrees/%s/locked", wt->id));
	free_worktrees(worktrees);
	return ret;
}

static void validate_no_submodules(const struct worktree *wt)
{
	struct index_state istate = { NULL };
	int i, found_submodules = 0;

	if (read_index_from(&istate, worktree_git_path(wt, "index"),
			    get_worktree_git_dir(wt)) > 0) {
		for (i = 0; i < istate.cache_nr; i++) {
			struct cache_entry *ce = istate.cache[i];

			if (S_ISGITLINK(ce->ce_mode)) {
				found_submodules = 1;
				break;
			}
		}
	}
	discard_index(&istate);

	if (found_submodules)
		die(_("working trees containing submodules cannot be moved or removed"));
}

static int move_worktree(int ac, const char **av, const char *prefix)
{
	struct option options[] = {
		OPT_END()
	};
	struct worktree **worktrees, *wt;
	struct strbuf dst = STRBUF_INIT;
	struct strbuf errmsg = STRBUF_INIT;
	const char *reason;
	char *path;

	ac = parse_options(ac, av, prefix, options, worktree_usage, 0);
	if (ac != 2)
		usage_with_options(worktree_usage, options);

	path = prefix_filename(prefix, av[1]);
	strbuf_addstr(&dst, path);
	free(path);

	worktrees = get_worktrees(0);
	wt = find_worktree(worktrees, prefix, av[0]);
	if (!wt)
		die(_("'%s' is not a working tree"), av[0]);
	if (is_main_worktree(wt))
		die(_("'%s' is a main working tree"), av[0]);
	if (is_directory(dst.buf)) {
		const char *sep = find_last_dir_sep(wt->path);

		if (!sep)
			die(_("could not figure out destination name from '%s'"),
			    wt->path);
		strbuf_trim_trailing_dir_sep(&dst);
		strbuf_addstr(&dst, sep);
	}
	if (file_exists(dst.buf))
		die(_("target '%s' already exists"), dst.buf);

	validate_no_submodules(wt);

	reason = is_worktree_locked(wt);
	if (reason) {
		if (*reason)
			die(_("cannot move a locked working tree, lock reason: %s"),
			    reason);
		die(_("cannot move a locked working tree"));
	}
	if (validate_worktree(wt, &errmsg, 0))
		die(_("validation failed, cannot move working tree: %s"),
		    errmsg.buf);
	strbuf_release(&errmsg);

	if (rename(wt->path, dst.buf) == -1)
		die_errno(_("failed to move '%s' to '%s'"), wt->path, dst.buf);

	update_worktree_location(wt, dst.buf);

	strbuf_release(&dst);
	free_worktrees(worktrees);
	return 0;
}

/*
 * Note, "git status --porcelain" is used to determine if it's safe to
 * delete a whole worktree. "git status" does not ignore user
 * configuration, so if a normal "git status" shows "clean" for the
 * user, then it's ok to remove it.
 *
 * This assumption may be a bad one. We may want to ignore
 * (potentially bad) user settings and only delete a worktree when
 * it's absolutely safe to do so from _our_ point of view because we
 * know better.
 */
static void check_clean_worktree(struct worktree *wt,
				 const char *original_path)
{
	struct argv_array child_env = ARGV_ARRAY_INIT;
	struct child_process cp;
	char buf[1];
	int ret;

	/*
	 * Until we sort this out, all submodules are "dirty" and
	 * will abort this function.
	 */
	validate_no_submodules(wt);

	argv_array_pushf(&child_env, "%s=%s/.git",
			 GIT_DIR_ENVIRONMENT, wt->path);
	argv_array_pushf(&child_env, "%s=%s",
			 GIT_WORK_TREE_ENVIRONMENT, wt->path);
	memset(&cp, 0, sizeof(cp));
	argv_array_pushl(&cp.args, "status",
			 "--porcelain", "--ignore-submodules=none",
			 NULL);
	cp.env = child_env.argv;
	cp.git_cmd = 1;
	cp.dir = wt->path;
	cp.out = -1;
	ret = start_command(&cp);
	if (ret)
		die_errno(_("failed to run 'git status' on '%s'"),
			  original_path);
	ret = xread(cp.out, buf, sizeof(buf));
	if (ret)
		die(_("'%s' is dirty, use --force to delete it"),
		    original_path);
	close(cp.out);
	ret = finish_command(&cp);
	if (ret)
		die_errno(_("failed to run 'git status' on '%s', code %d"),
			  original_path, ret);
}

static int delete_git_work_tree(struct worktree *wt)
{
	struct strbuf sb = STRBUF_INIT;
	int ret = 0;

	strbuf_addstr(&sb, wt->path);
	if (remove_dir_recursively(&sb, 0)) {
		error_errno(_("failed to delete '%s'"), sb.buf);
		ret = -1;
	}
	strbuf_release(&sb);
	return ret;
}

static int delete_git_dir(struct worktree *wt)
{
	struct strbuf sb = STRBUF_INIT;
	int ret = 0;

	strbuf_addstr(&sb, git_common_path("worktrees/%s", wt->id));
	if (remove_dir_recursively(&sb, 0)) {
		error_errno(_("failed to delete '%s'"), sb.buf);
		ret = -1;
	}
	strbuf_release(&sb);
	return ret;
}

static int remove_worktree(int ac, const char **av, const char *prefix)
{
	int force = 0;
	struct option options[] = {
		OPT_BOOL(0, "force", &force,
			 N_("force removing even if the worktree is dirty")),
		OPT_END()
	};
	struct worktree **worktrees, *wt;
	struct strbuf errmsg = STRBUF_INIT;
	const char *reason;
	int ret = 0;

	ac = parse_options(ac, av, prefix, options, worktree_usage, 0);
	if (ac != 1)
		usage_with_options(worktree_usage, options);

	worktrees = get_worktrees(0);
	wt = find_worktree(worktrees, prefix, av[0]);
	if (!wt)
		die(_("'%s' is not a working tree"), av[0]);
	if (is_main_worktree(wt))
		die(_("'%s' is a main working tree"), av[0]);
	reason = is_worktree_locked(wt);
	if (reason) {
		if (*reason)
			die(_("cannot remove a locked working tree, lock reason: %s"),
			    reason);
		die(_("cannot remove a locked working tree"));
	}
	if (validate_worktree(wt, &errmsg, WT_VALIDATE_WORKTREE_MISSING_OK))
		die(_("validation failed, cannot remove working tree: %s"),
		    errmsg.buf);
	strbuf_release(&errmsg);

	if (file_exists(wt->path)) {
		if (!force)
			check_clean_worktree(wt, av[0]);

		ret |= delete_git_work_tree(wt);
	}
	/*
	 * continue on even if ret is non-zero, there's no going back
	 * from here.
	 */
	ret |= delete_git_dir(wt);

	free_worktrees(worktrees);
	return ret;
}

int cmd_worktree(int ac, const char **av, const char *prefix)
{
	struct option options[] = {
		OPT_END()
	};

	git_config(git_worktree_config, NULL);

	if (ac < 2)
		usage_with_options(worktree_usage, options);
	if (!prefix)
		prefix = "";
	if (!strcmp(av[1], "add"))
		return add(ac - 1, av + 1, prefix);
	if (!strcmp(av[1], "prune"))
		return prune(ac - 1, av + 1, prefix);
	if (!strcmp(av[1], "list"))
		return list(ac - 1, av + 1, prefix);
	if (!strcmp(av[1], "lock"))
		return lock_worktree(ac - 1, av + 1, prefix);
	if (!strcmp(av[1], "unlock"))
		return unlock_worktree(ac - 1, av + 1, prefix);
	if (!strcmp(av[1], "move"))
		return move_worktree(ac - 1, av + 1, prefix);
	if (!strcmp(av[1], "remove"))
		return remove_worktree(ac - 1, av + 1, prefix);
	usage_with_options(worktree_usage, options);
}
