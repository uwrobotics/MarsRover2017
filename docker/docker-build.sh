#!/bin/bash

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

cp -r ${DIR}/scripts ${DIR}/docker/${1}/scripts

docker build -t uwrt/ubuntu:${1} ${DIR}/docker/${1}

RES=$?

rm -r ${DIR}/docker/${1}/scripts

if [ ${RES} -ne 0 ]; then
    echo "Error: ${1} docker build failed"
    exit 1
fi
