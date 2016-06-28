#!/usr/bin/env python

import time, sys, os
if __name__ == '__main__':

    if (not os.path.exists('../data/')):
        os.mkdir('../data/')

    msgFile = open('../msg/Command.msg', 'r')
    corpusFile = open('../data/commands.corpus', 'w')
    
    msgLine = msgFile.readline()
    while (msgLine != ''):
        msgLine = msgFile.readline()
        if (msgLine.find('=') != -1):
            lineParts = msgLine.split("=")
            commandStr = lineParts[len(lineParts)-1]
            while(commandStr[0] == ' '):
                commandStr = commandStr[1:(len(commandStr))]
            while(commandStr[len(commandStr)-1] == ' ' or commandStr[len(commandStr)-1] == '\n'):
                commandStr = commandStr[0:(len(commandStr)-1)]
            #commandStr = commandStr[1:(len(commandStr)-1)]
            if(commandStr != 'unrecognized'):
                corpusFile.write(commandStr+'\n');
    corpusFile.close()
    msgFile.close()
