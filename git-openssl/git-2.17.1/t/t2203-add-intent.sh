#!/bin/sh

test_description='Intent to add'

. ./test-lib.sh

test_expect_success 'intent to add' '
	test_commit 1 &&
	git rm 1.t &&
	echo hello >1.t &&
	echo hello >file &&
	echo hello >elif &&
	git add -N file &&
	git add elif &&
	git add -N 1.t
'

test_expect_success 'git status' '
	git status --porcelain | grep -v actual >actual &&
	cat >expect <<-\EOF &&
	DA 1.t
	A  elif
	 A file
	EOF
	test_cmp expect actual
'

test_expect_success 'git status with porcelain v2' '
	git status --porcelain=v2 | grep -v "^?" >actual &&
	nam1=d00491fd7e5bb6fa28c517a0bb32b8b506539d4d &&
	nam2=ce013625030ba8dba906f756967f9e9ca394464a &&
	cat >expect <<-EOF &&
	1 DA N... 100644 000000 100644 $nam1 $_z40 1.t
	1 A. N... 000000 100644 100644 $_z40 $nam2 elif
	1 .A N... 000000 000000 100644 $_z40 $_z40 file
	EOF
	test_cmp expect actual
'

test_expect_success 'check result of "add -N"' '
	git ls-files -s file >actual &&
	empty=$(git hash-object --stdin </dev/null) &&
	echo "100644 $empty 0	file" >expect &&
	test_cmp expect actual
'

test_expect_success 'intent to add is just an ordinary empty blob' '
	git add -u &&
	git ls-files -s file >actual &&
	git ls-files -s elif | sed -e "s/elif/file/" >expect &&
	test_cmp expect actual
'

test_expect_success 'intent to add does not clobber existing paths' '
	git add -N file elif &&
	empty=$(git hash-object --stdin </dev/null) &&
	git ls-files -s >actual &&
	! grep "$empty" actual
'

test_expect_success 'i-t-a entry is simply ignored' '
	test_tick &&
	git commit -a -m initial &&
	git reset --hard &&

	echo xyzzy >rezrov &&
	echo frotz >nitfol &&
	git add rezrov &&
	git add -N nitfol &&
	git commit -m second &&
	test $(git ls-tree HEAD -- nitfol | wc -l) = 0 &&
	test $(git diff --name-only HEAD -- nitfol | wc -l) = 1 &&
	test $(git diff --name-only --ita-invisible-in-index HEAD -- nitfol | wc -l) = 0 &&
	test $(git diff --name-only --ita-invisible-in-index -- nitfol | wc -l) = 1
'

test_expect_success 'can commit with an unrelated i-t-a entry in index' '
	git reset --hard &&
	echo bozbar >rezrov &&
	echo frotz >nitfol &&
	git add rezrov &&
	git add -N nitfol &&
	git commit -m partial rezrov
'

test_expect_success 'can "commit -a" with an i-t-a entry' '
	git reset --hard &&
	: >nitfol &&
	git add -N nitfol &&
	git commit -a -m all
'

test_expect_success 'cache-tree invalidates i-t-a paths' '
	git reset --hard &&
	mkdir dir &&
	: >dir/foo &&
	git add dir/foo &&
	git commit -m foo &&

	: >dir/bar &&
	git add -N dir/bar &&
	git diff --cached --name-only >actual &&
	echo dir/bar >expect &&
	test_cmp expect actual &&

	git write-tree >/dev/null &&

	git diff --cached --name-only >actual &&
	echo dir/bar >expect &&
	test_cmp expect actual
'

test_expect_success 'cache-tree does not ignore dir that has i-t-a entries' '
	git init ita-in-dir &&
	(
		cd ita-in-dir &&
		mkdir 2 &&
		for f in 1 2/1 2/2 3
		do
			echo "$f" >"$f"
		done &&
		git add 1 2/2 3 &&
		git add -N 2/1 &&
		git commit -m committed &&
		git ls-tree -r HEAD >actual &&
		grep 2/2 actual
	)
'

test_expect_success 'cache-tree does skip dir that becomes empty' '
	rm -fr ita-in-dir &&
	git init ita-in-dir &&
	(
		cd ita-in-dir &&
		mkdir -p 1/2/3 &&
		echo 4 >1/2/3/4 &&
		git add -N 1/2/3/4 &&
		git write-tree >actual &&
		echo $EMPTY_TREE >expected &&
		test_cmp expected actual
	)
'

test_expect_success 'commit: ita entries ignored in empty initial commit check' '
	git init empty-initial-commit &&
	(
		cd empty-initial-commit &&
		: >one &&
		git add -N one &&
		test_must_fail git commit -m nothing-new-here
	)
'

test_expect_success 'commit: ita entries ignored in empty commit check' '
	git init empty-subsequent-commit &&
	(
		cd empty-subsequent-commit &&
		test_commit one &&
		: >two &&
		git add -N two &&
		test_must_fail git commit -m nothing-new-here
	)
'

test_expect_success 'rename detection finds the right names' '
	git init rename-detection &&
	(
		cd rename-detection &&
		echo contents >first &&
		git add first &&
		git commit -m first &&
		mv first third &&
		git add -N third &&

		git status | grep -v "^?" >actual.1 &&
		test_i18ngrep "renamed: *first -> third" actual.1 &&

		git status --porcelain | grep -v "^?" >actual.2 &&
		cat >expected.2 <<-\EOF &&
		 R first -> third
		EOF
		test_cmp expected.2 actual.2 &&

		hash=12f00e90b6ef79117ce6e650416b8cf517099b78 &&
		git status --porcelain=v2 | grep -v "^?" >actual.3 &&
		cat >expected.3 <<-EOF &&
		2 .R N... 100644 100644 100644 $hash $hash R100 third	first
		EOF
		test_cmp expected.3 actual.3
	)
'

test_expect_success 'double rename detection in status' '
	git init rename-detection-2 &&
	(
		cd rename-detection-2 &&
		echo contents >first &&
		git add first &&
		git commit -m first &&
		git mv first second &&
		mv second third &&
		git add -N third &&

		git status | grep -v "^?" >actual.1 &&
		test_i18ngrep "renamed: *first -> second" actual.1 &&
		test_i18ngrep "renamed: *second -> third" actual.1 &&

		git status --porcelain | grep -v "^?" >actual.2 &&
		cat >expected.2 <<-\EOF &&
		R  first -> second
		 R second -> third
		EOF
		test_cmp expected.2 actual.2 &&

		hash=12f00e90b6ef79117ce6e650416b8cf517099b78 &&
		git status --porcelain=v2 | grep -v "^?" >actual.3 &&
		cat >expected.3 <<-EOF &&
		2 R. N... 100644 100644 100644 $hash $hash R100 second	first
		2 .R N... 100644 100644 100644 $hash $hash R100 third	second
		EOF
		test_cmp expected.3 actual.3
	)
'

test_done

