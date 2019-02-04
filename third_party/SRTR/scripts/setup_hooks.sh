#!/bin/bash
ln -s ../../scripts/commit.sh ./.git/hooks/pre-commit
ln -s ../../scripts/push.sh ./.git/hooks/pre-push
echo "What is your UMass email that you include in the copyright header?"
read email
echo "$email" > .git/minutebotemail
