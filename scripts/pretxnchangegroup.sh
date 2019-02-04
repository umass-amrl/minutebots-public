#!/bin/bash
echo -e "\e[34m\e[1mSTARTING GOOGLE C++ LINT CHECK...\e[0m"


setup_mail_vars() {
    #Variables used in email generation
    changelist=$(hg log -p -r $HG_NODE:)
    commit_num=$(hg log --rev $HG_NODE: --template "{rev}, ")
    # Removes trailing ", "
    commit_num=${commit_num::-2}
    tldr=$(hg log --rev $HG_NODE: --template "{rev}: {desc}\n")
}

send_mail () {
    #http://superuser.com/questions/235738/how-do-i-substitute-environment-variables-when-i-ouput-a-file
     mail_template=$(EOF=EOF_$RANDOM; eval echo "\"$(cat <<$EOF
$(<./scripts/email_incoming.txt)
$EOF
)\"")
     echo "Sending mail for commit number $commit_num..."
     echo "$mail_template" > $temp_dir/email.txt
     sendmail "amrl@cs.umass.edu" < $temp_dir/email.txt
}

cleanup () {
  rm -rf $temp_dir
}

#############
# Main Execution
#############

setup_mail_vars

# NOTE(joydeepb): Not using this since it did not seem to list all the affected
# files.
# changedFiles=$(hg log -r $HG_NODE --template="{files}\n" --include "src/*")
# echo "Changed files: $changedFiles"

# As per https://www.selenic.com/mercurial/hgrc.5.html#hooks, gets the changeset in
# $HG_NODE, and then gets the branch name.
branch_name=$(hg log --rev $HG_NODE --template "{branch}")
echo -e "\e[34m\e[1mBranch Name:\e[0m $branch_name"

# Create a temp clone of the repository.
temp_dir=$(mktemp -u --tmpdir robocup-ssl-mercurial-XXXXX)
hg clone . -r $branch_name $temp_dir > /dev/null
pushd $temp_dir > /dev/null
source_files=`find src/ -type f -name "*.c*" -o -name "*.h*"`

scripts/cpplint.py $source_files >$temp_dir/linter_output.txt 2>&1
res=$?
cat $temp_dir/linter_output.txt
linter_output=$(cat $temp_dir/linter_output.txt)

if [ "$res" -ne 0 ];then
  echo -e "\033[1m\e[31mRemote Lint Failures!!!\e[0m";
  status="FAILED"
  # NOTE(kvedder): Removes failing linter emails. Uncomment to re-add feature.
  # send_mail
  cleanup
  exit 1;
fi

echo -e "\033[1m\e[32mRemote Lint Passed...\e[0m";
status="PASSED"
send_mail
echo "Updating remote repo"
hg up
cleanup
exit 0
