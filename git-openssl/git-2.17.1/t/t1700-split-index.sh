#!/bin/sh

test_description='split index mode tests'

. ./test-lib.sh

# We need total control of index splitting here
sane_unset GIT_TEST_SPLIT_INDEX
sane_unset GIT_FSMONITOR_TEST

test_expect_success 'enable split index' '
	git config splitIndex.maxPercentChange 100 &&
	git update-index --split-index &&
	test-dump-split-index .git/index >actual &&
	indexversion=$(test-index-version <.git/index) &&
	if test "$indexversion" = "4"
	then
		own=432ef4b63f32193984f339431fd50ca796493569
		base=508851a7f0dfa8691e9f69c7f055865389012491
	else
		own=8299b0bcd1ac364e5f1d7768efb62fa2da79a339
		base=39d890139ee5356c7ef572216cebcd27aa41f9df
	fi &&
	cat >expect <<-EOF &&
	own $own
	base $base
	replacements:
	deletions:
	EOF
	test_cmp expect actual
'

test_expect_success 'add one file' '
	: >one &&
	git update-index --add one &&
	git ls-files --stage >ls-files.actual &&
	cat >ls-files.expect <<-EOF &&
	100644 $EMPTY_BLOB 0	one
	EOF
	test_cmp ls-files.expect ls-files.actual &&

	test-dump-split-index .git/index | sed "/^own/d" >actual &&
	cat >expect <<-EOF &&
	base $base
	100644 $EMPTY_BLOB 0	one
	replacements:
	deletions:
	EOF
	test_cmp expect actual
'

test_expect_success 'disable split index' '
	git update-index --no-split-index &&
	git ls-files --stage >ls-files.actual &&
	cat >ls-files.expect <<-EOF &&
	100644 $EMPTY_BLOB 0	one
	EOF
	test_cmp ls-files.expect ls-files.actual &&

	BASE=$(test-dump-split-index .git/index | grep "^own" | sed "s/own/base/") &&
	test-dump-split-index .git/index | sed "/^own/d" >actual &&
	cat >expect <<-EOF &&
	not a split index
	EOF
	test_cmp expect actual
'

test_expect_success 'enable split index again, "one" now belongs to base index"' '
	git update-index --split-index &&
	git ls-files --stage >ls-files.actual &&
	cat >ls-files.expect <<-EOF &&
	100644 $EMPTY_BLOB 0	one
	EOF
	test_cmp ls-files.expect ls-files.actual &&

	test-dump-split-index .git/index | sed "/^own/d" >actual &&
	cat >expect <<-EOF &&
	$BASE
	replacements:
	deletions:
	EOF
	test_cmp expect actual
'

test_expect_success 'modify original file, base index untouched' '
	echo modified >one &&
	git update-index one &&
	git ls-files --stage >ls-files.actual &&
	cat >ls-files.expect <<-EOF &&
	100644 2e0996000b7e9019eabcad29391bf0f5c7702f0b 0	one
	EOF
	test_cmp ls-files.expect ls-files.actual &&

	test-dump-split-index .git/index | sed "/^own/d" >actual &&
	q_to_tab >expect <<-EOF &&
	$BASE
	100644 2e0996000b7e9019eabcad29391bf0f5c7702f0b 0Q
	replacements: 0
	deletions:
	EOF
	test_cmp expect actual
'

test_expect_success 'add another file, which stays index' '
	: >two &&
	git update-index --add two &&
	git ls-files --stage >ls-files.actual &&
	cat >ls-files.expect <<-EOF &&
	100644 2e0996000b7e9019eabcad29391bf0f5c7702f0b 0	one
	100644 $EMPTY_BLOB 0	two
	EOF
	test_cmp ls-files.expect ls-files.actual &&

	test-dump-split-index .git/index | sed "/^own/d" >actual &&
	q_to_tab >expect <<-EOF &&
	$BASE
	100644 2e0996000b7e9019eabcad29391bf0f5c7702f0b 0Q
	100644 $EMPTY_BLOB 0	two
	replacements: 0
	deletions:
	EOF
	test_cmp expect actual
'

test_expect_success 'remove file not in base index' '
	git update-index --force-remove two &&
	git ls-files --stage >ls-files.actual &&
	cat >ls-files.expect <<-EOF &&
	100644 2e0996000b7e9019eabcad29391bf0f5c7702f0b 0	one
	EOF
	test_cmp ls-files.expect ls-files.actual &&

	test-dump-split-index .git/index | sed "/^own/d" >actual &&
	q_to_tab >expect <<-EOF &&
	$BASE
	100644 2e0996000b7e9019eabcad29391bf0f5c7702f0b 0Q
	replacements: 0
	deletions:
	EOF
	test_cmp expect actual
'

test_expect_success 'remove file in base index' '
	git update-index --force-remove one &&
	git ls-files --stage >ls-files.actual &&
	cat >ls-files.expect <<-EOF &&
	EOF
	test_cmp ls-files.expect ls-files.actual &&

	test-dump-split-index .git/index | sed "/^own/d" >actual &&
	cat >expect <<-EOF &&
	$BASE
	replacements:
	deletions: 0
	EOF
	test_cmp expect actual
'

test_expect_success 'add original file back' '
	: >one &&
	git update-index --add one &&
	git ls-files --stage >ls-files.actual &&
	cat >ls-files.expect <<-EOF &&
	100644 $EMPTY_BLOB 0	one
	EOF
	test_cmp ls-files.expect ls-files.actual &&

	test-dump-split-index .git/index | sed "/^own/d" >actual &&
	cat >expect <<-EOF &&
	$BASE
	100644 $EMPTY_BLOB 0	one
	replacements:
	deletions: 0
	EOF
	test_cmp expect actual
'

test_expect_success 'add new file' '
	: >two &&
	git update-index --add two &&
	git ls-files --stage >actual &&
	cat >expect <<-EOF &&
	100644 $EMPTY_BLOB 0	one
	100644 $EMPTY_BLOB 0	two
	EOF
	test_cmp expect actual
'

test_expect_success 'unify index, two files remain' '
	git update-index --no-split-index &&
	git ls-files --stage >ls-files.actual &&
	cat >ls-files.expect <<-EOF &&
	100644 $EMPTY_BLOB 0	one
	100644 $EMPTY_BLOB 0	two
	EOF
	test_cmp ls-files.expect ls-files.actual &&

	test-dump-split-index .git/index | sed "/^own/d" >actual &&
	cat >expect <<-EOF &&
	not a split index
	EOF
	test_cmp expect actual
'

test_expect_success 'rev-parse --shared-index-path' '
	test_create_repo split-index &&
	(
		cd split-index &&
		git update-index --split-index &&
		echo .git/sharedindex* >expect &&
		git rev-parse --shared-index-path >actual &&
		test_cmp expect actual &&
		mkdir subdirectory &&
		cd subdirectory &&
		echo ../.git/sharedindex* >expect &&
		git rev-parse --shared-index-path >actual &&
		test_cmp expect actual
	)
'

test_expect_success 'set core.splitIndex config variable to true' '
	git config core.splitIndex true &&
	: >three &&
	git update-index --add three &&
	git ls-files --stage >ls-files.actual &&
	cat >ls-files.expect <<-EOF &&
	100644 e69de29bb2d1d6434b8b29ae775ad8c2e48c5391 0	one
	100644 e69de29bb2d1d6434b8b29ae775ad8c2e48c5391 0	three
	100644 e69de29bb2d1d6434b8b29ae775ad8c2e48c5391 0	two
	EOF
	test_cmp ls-files.expect ls-files.actual &&
	BASE=$(test-dump-split-index .git/index | grep "^base") &&
	test-dump-split-index .git/index | sed "/^own/d" >actual &&
	cat >expect <<-EOF &&
	$BASE
	replacements:
	deletions:
	EOF
	test_cmp expect actual
'

test_expect_success 'set core.splitIndex config variable to false' '
	git config core.splitIndex false &&
	git update-index --force-remove three &&
	git ls-files --stage >ls-files.actual &&
	cat >ls-files.expect <<-EOF &&
	100644 e69de29bb2d1d6434b8b29ae775ad8c2e48c5391 0	one
	100644 e69de29bb2d1d6434b8b29ae775ad8c2e48c5391 0	two
	EOF
	test_cmp ls-files.expect ls-files.actual &&
	test-dump-split-index .git/index | sed "/^own/d" >actual &&
	cat >expect <<-EOF &&
	not a split index
	EOF
	test_cmp expect actual
'

test_expect_success 'set core.splitIndex config variable to true' '
	git config core.splitIndex true &&
	: >three &&
	git update-index --add three &&
	BASE=$(test-dump-split-index .git/index | grep "^base") &&
	test-dump-split-index .git/index | sed "/^own/d" >actual &&
	cat >expect <<-EOF &&
	$BASE
	replacements:
	deletions:
	EOF
	test_cmp expect actual &&
	: >four &&
	git update-index --add four &&
	test-dump-split-index .git/index | sed "/^own/d" >actual &&
	cat >expect <<-EOF &&
	$BASE
	100644 e69de29bb2d1d6434b8b29ae775ad8c2e48c5391 0	four
	replacements:
	deletions:
	EOF
	test_cmp expect actual
'

test_expect_success 'check behavior with splitIndex.maxPercentChange unset' '
	git config --unset splitIndex.maxPercentChange &&
	: >five &&
	git update-index --add five &&
	BASE=$(test-dump-split-index .git/index | grep "^base") &&
	test-dump-split-index .git/index | sed "/^own/d" >actual &&
	cat >expect <<-EOF &&
	$BASE
	replacements:
	deletions:
	EOF
	test_cmp expect actual &&
	: >six &&
	git update-index --add six &&
	test-dump-split-index .git/index | sed "/^own/d" >actual &&
	cat >expect <<-EOF &&
	$BASE
	100644 e69de29bb2d1d6434b8b29ae775ad8c2e48c5391 0	six
	replacements:
	deletions:
	EOF
	test_cmp expect actual
'

test_expect_success 'check splitIndex.maxPercentChange set to 0' '
	git config splitIndex.maxPercentChange 0 &&
	: >seven &&
	git update-index --add seven &&
	BASE=$(test-dump-split-index .git/index | grep "^base") &&
	test-dump-split-index .git/index | sed "/^own/d" >actual &&
	cat >expect <<-EOF &&
	$BASE
	replacements:
	deletions:
	EOF
	test_cmp expect actual &&
	: >eight &&
	git update-index --add eight &&
	BASE=$(test-dump-split-index .git/index | grep "^base") &&
	test-dump-split-index .git/index | sed "/^own/d" >actual &&
	cat >expect <<-EOF &&
	$BASE
	replacements:
	deletions:
	EOF
	test_cmp expect actual
'

test_expect_success 'shared index files expire after 2 weeks by default' '
	: >ten &&
	git update-index --add ten &&
	test $(ls .git/sharedindex.* | wc -l) -gt 2 &&
	just_under_2_weeks_ago=$((5-14*86400)) &&
	test-chmtime =$just_under_2_weeks_ago .git/sharedindex.* &&
	: >eleven &&
	git update-index --add eleven &&
	test $(ls .git/sharedindex.* | wc -l) -gt 2 &&
	just_over_2_weeks_ago=$((-1-14*86400)) &&
	test-chmtime =$just_over_2_weeks_ago .git/sharedindex.* &&
	: >twelve &&
	git update-index --add twelve &&
	test $(ls .git/sharedindex.* | wc -l) -le 2
'

test_expect_success 'check splitIndex.sharedIndexExpire set to 16 days' '
	git config splitIndex.sharedIndexExpire "16.days.ago" &&
	test-chmtime =$just_over_2_weeks_ago .git/sharedindex.* &&
	: >thirteen &&
	git update-index --add thirteen &&
	test $(ls .git/sharedindex.* | wc -l) -gt 2 &&
	just_over_16_days_ago=$((-1-16*86400)) &&
	test-chmtime =$just_over_16_days_ago .git/sharedindex.* &&
	: >fourteen &&
	git update-index --add fourteen &&
	test $(ls .git/sharedindex.* | wc -l) -le 2
'

test_expect_success 'check splitIndex.sharedIndexExpire set to "never" and "now"' '
	git config splitIndex.sharedIndexExpire never &&
	just_10_years_ago=$((-365*10*86400)) &&
	test-chmtime =$just_10_years_ago .git/sharedindex.* &&
	: >fifteen &&
	git update-index --add fifteen &&
	test $(ls .git/sharedindex.* | wc -l) -gt 2 &&
	git config splitIndex.sharedIndexExpire now &&
	just_1_second_ago=-1 &&
	test-chmtime =$just_1_second_ago .git/sharedindex.* &&
	: >sixteen &&
	git update-index --add sixteen &&
	test $(ls .git/sharedindex.* | wc -l) -le 2
'

while read -r mode modebits
do
	test_expect_success POSIXPERM "split index respects core.sharedrepository $mode" '
		# Remove existing shared index files
		git config core.splitIndex false &&
		git update-index --force-remove one &&
		rm -f .git/sharedindex.* &&
		# Create one new shared index file
		git config core.sharedrepository "$mode" &&
		git config core.splitIndex true &&
		: >one &&
		git update-index --add one &&
		echo "$modebits" >expect &&
		test_modebits .git/index >actual &&
		test_cmp expect actual &&
		shared=$(ls .git/sharedindex.*) &&
		case "$shared" in
		*" "*)
			# we have more than one???
			false ;;
		*)
			test_modebits "$shared" >actual &&
			test_cmp expect actual ;;
		esac
	'
done <<\EOF
0666 -rw-rw-rw-
0642 -rw-r---w-
EOF

test_expect_success POSIXPERM,SANITY 'graceful handling when splitting index is not allowed' '
	test_create_repo ro &&
	(
		cd ro &&
		test_commit initial &&
		git update-index --split-index &&
		test -f .git/sharedindex.*
	) &&
	cp ro/.git/index new-index &&
	test_when_finished "chmod u+w ro/.git" &&
	chmod u-w ro/.git &&
	GIT_INDEX_FILE="$(pwd)/new-index" git -C ro update-index --split-index &&
	chmod u+w ro/.git &&
	rm ro/.git/sharedindex.* &&
	GIT_INDEX_FILE=new-index git ls-files >actual &&
	echo initial.t >expected &&
	test_cmp expected actual
'

test_expect_success 'writing split index with null sha1 does not write cache tree' '
	git config core.splitIndex true &&
	git config splitIndex.maxPercentChange 0 &&
	git commit -m "commit" &&
	{
		git ls-tree HEAD &&
		printf "160000 commit $_z40\\tbroken\\n"
	} >broken-tree &&
	echo "add broken entry" >msg &&

	tree=$(git mktree <broken-tree) &&
	test_tick &&
	commit=$(git commit-tree $tree -p HEAD <msg) &&
	git update-ref HEAD "$commit" &&
	GIT_ALLOW_NULL_SHA1=1 git reset --hard &&
	(test-dump-cache-tree >cache-tree.out || true) &&
	test_line_count = 0 cache-tree.out
'

test_done
