#!/bin/bash
curl -o- https://raw.githubusercontent.com/creationix/nvm/v0.31.6/install.sh | bash
export NVM_DIR="$HOME/.nvm"
[ -s "$NVM_DIR/nvm.sh" ] && . "$NVM_DIR/nvm.sh"
nvm install node
npm install bower
npm install polymer-cli
./node_modules/bower/bin/bower install
