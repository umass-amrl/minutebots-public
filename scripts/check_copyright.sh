#!/bin/bash

check_year() {
    current_year=$(date +"%Y")
    if [[ $first_line != *"$current_year"* ]]
    then
        echo "$file_path: Current year is not found in the Copyright header!"
        echo "Header: $first_line"
        fixed_first_line=$(./scripts/edit_date.py "$first_line")
        echo "Fixed to: $fixed_first_line"
        sed -i "1 s/^.*$/$fixed_first_line/" "$file_path"
    fi
}

check_email() {
    if [  -f .git/minutebotemail ]; then
        email=$(cat .git/minutebotemail)
    else
        email=$(git config user.email)
    fi
    if [[ $five_lines != *"$email"* ]]
    then
        echo -e "\e[36m$file_path: Your email, $email, is not found in the Copyright header!\e[0m"
    fi
}
file_path=$1
full_file=$(cat $1)
if [ -z "$full_file" ]
then
    exit
fi
first_line=$(head -n 1 $1)
five_lines=$(head -n 5 $1)
read_exit_status=$?
if ((read_exit_status != 0))
then
    # Can't read file, quit.
    exit;
fi

check_year
check_email
