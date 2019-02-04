# Minutebots

Welcome to the UMass Minutebots repository!

To get started, please follow the instructions to [setup our codebase.](https://github.com/umass-amrl/minutebots-public/wiki/Initial-Repository-Setup)

To setup and run the SRTR Challenge Problem for BRASS: [SRTR Challenge](https://github.com/umass-amrl/minutebots-public/wiki/BRASS---SRTR-Challenge-Problem)

## License 

Our code is licensed under LGPL 3.0. Please do not check in code or use libraries that are licensed under GPL
or other viral licenses; we would like to retain the LGPL license.

## Debug

To `valgrind` the codebase, you need to add supressions for RE2. To use the pre-prepared supressions, you need to use

```
valgrind --error-exitcode=1 --track-origins=yes --soname-synonyms='somalloc=*tcmalloc*' --suppressions=scripts/valgrind/unittests.supp --leak-check=full
```
