#!/bin/bash
FILES=$(hg stat | grep "src/" | cut -d" " -f2)
if [ -z $FILES ]; then
  echo -e "\033[1m\e[32mNo Modified Files...\e[0m";
  exit
fi

./scripts/cpplint.py $FILES
ret=$?
if [ $ret -ne 0 ]; then
    echo -e "\033[1m\e[31mManual C++ Lint Failures!!!\e[0m";
    exit -1;
fi
./scripts/luacheck/bin/luacheck --config ./scripts/luacheck_config $(find src -name '*.lua')
ret=$?
if [ $ret -ne 0 ]; then
  echo -e "\033[1m\e[31mManual Lua Lint Failures!!!\e[0m";
  exit -1;
fi
echo -e "\033[1m\e[32mManual Lint Passed...\e[0m";
exit 0;
