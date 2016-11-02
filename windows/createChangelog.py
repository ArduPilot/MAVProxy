#!/usr/bin/env python
'''
Create a user readable changelog
Requires the gitpython package via pip install gitpython

Stephen Dade
November 2016
'''

from git import Repo
import os
import time

#Get the parent (..\) directory. ie the root MAVProxy dir
path = os.path.abspath(os.path.join(os.getcwd(), os.pardir))
repo = Repo(path)
assert not repo.bare

#get list of all commits
all_commits = list(repo.iter_commits('master'))

#open the changelog for writing
f = open("changelog.txt","w") 

#go through all the commits
for comm in all_commits:
    #if it's a version raise, add a special message
    if "raise version" in comm.message:
        commit_date = time.strftime("%d-%m-%Y", time.gmtime(comm.committed_date))
        tree = comm.tree
        #get setup.py and grab the version number from the file
        blob = tree['setup.py']
        data = blob.data_stream.read()
        curversion = ""
        for line in data.split('\n'):
            if "version = " in line: 
                curversion =  line[11:len(line)-1]
                break
            
        f.write("\n")
        f.write("MAVProxy " + curversion + " (" + commit_date + ")\n")
    else:
        #just print the summary (1st line) of the commit message
        comm.message.split('\n', 1)[0]
        f.write("-" + comm.summary + "\n")
    
f.close()
print("Done")
