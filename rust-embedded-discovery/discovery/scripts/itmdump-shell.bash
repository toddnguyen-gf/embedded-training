#!/bin/bash

set -euxo pipefail

cd /tmp && touch itm.txt
itmdump -F -f itm.txt
