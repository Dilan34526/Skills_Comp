import sys
import os
import time
os.system('prosV5 make')
print('\n')
os.system('powershell "prosV5 upload --slot ' + sys.argv[1] + "" + ' | tee "upload.txt"')
file = open("upload.txt", "r", encoding = "UTF-16")
string = file.readline()
i = 6
if "No v5 ports were" in string:
    print("Hey")
    while i > 0:
        os.system('prosV5 lsusb --target v5> upload.txt')
        file = open("upload.txt", "r")
        string = file.readline()
        string.replace("\x00", "")
        if "no connected" not in string:
            i = -1
        file.close()
        i = i - 1
        time.sleep(0.05)
        slot = sys.argv[1]
        line = ""
    if "no connected" not in string:
            cmdline = "prosV5 upload --slot " + sys.argv[1]
            line = 'cscript MessageBox.vbs '  + '"Uploaded to Slot ' + sys.argv[1] + '"'
            os.system(cmdline)
            os.system(line)
    else:
        line = 'cscript MessageBox.vbs ' + '"Timed Out"'
        os.system(line)
else:
    line = 'cscript MessageBox.vbs '  + '"Uploaded to Slot ' + sys.argv[1] + '"'
    os.system(line)
