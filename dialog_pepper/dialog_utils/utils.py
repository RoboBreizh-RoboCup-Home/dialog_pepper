import numpy as np
import sqlite3
from ament_index_python.packages import get_package_share_directory
import os

#########################################################################################
# TERMINAL color
#########################################################################################
W = '\033[0m'  # white (normal)
R = '\033[31m'  # red
G = '\033[32m'  # green
O = '\033[33m'  # orange
B = '\033[34m'  # blue
P = '\033[35m'  # purple
CYAN = '\033[96m'

DB_PATH = "/home/nao/robobreizh/roboBreizhDb.db"

def get_pkg_path():
    # get current path of this script
    current_path = os.path.dirname(os.path.realpath(__file__))
    current_path = os.path.join(current_path, '../..')
    return(current_path)

def getvalue():
    """
    Write the transcript from the speech to text in the table dialog
    """
    conn = sqlite3.connect(DB_PATH)
    cur = conn.cursor()
    cur.execute("SELECT transcript FROM speech ORDER BY speech.id DESC LIMIT 1")
    value = cur.fetchall()
    if value == []:
        conn.close()
        return ""
    else:
        conn.close()
        return str(value[0][0])

def removeValue():
    """
    Write the transcript from the speech to text in the table dialog
    """
    conn = sqlite3.connect(DB_PATH)
    cur = conn.cursor()
    cur.execute("DELETE FROM speech")
    conn.commit()
    conn.close()

def writeValue(transcript):
    """
    Write the transcript from the speech to text in the table dialog
    """
    conn = sqlite3.connect(DB_PATH)
    cur = conn.cursor()
    cur.execute("INSERT INTO speech(transcript) VALUES(?)",[transcript])
    conn.commit()
    conn.close()

def isBooleanInDBTrue():
    """
    Get the boolean value of id one to check if there is a request for sound processing
    """
    conn = sqlite3.connect(DB_PATH)
    cur = conn.cursor()
    cur.execute("SELECT run FROM dialog WHERE id = 1")

    rows = cur.fetchall()

    if rows[0][0] == 1:
        conn.close()
        return  True
    conn.close()
    return False

def setBooleanInDBFalse():
    """
    set the process as done via the boolean value in the db
    """
    conn = sqlite3.connect(DB_PATH)
    cur = conn.cursor()
    cur.execute("update dialog set run = 0 where id = 1")
    conn.commit()
    conn.close()

def setBooleanInDBTrue():
    """
    set the process as done via the boolean value in the db
    """
    conn = sqlite3.connect(DB_PATH)
    cur = conn.cursor()
    r = cur.execute("update dialog set run = 1 where id = 1")
    conn.commit()
    conn.close()

def float_to_pcm(myrecording, dtype):
    myrecording = np.asarray(myrecording)
    i = np.iinfo(dtype)
    abs_max = 2 ** (i.bits - 1)
    offset = i.min + abs_max
    return (myrecording * abs_max + offset).clip(i.min, i.max).astype(dtype)

def convert_str_to_int(data):
    signedData = []
    ind = 0

    for i in range(0, int(len(data)/2)):
        signedData.append(data[ind]+data[ind+1]*256)

        ind = ind + 2

    for i in range(0, int(len(signedData))):
        if signedData[i] >= 32768:
            signedData[i] = signedData[i]-65536

    for i in range(0, int(len(signedData))):
        signedData[i] = signedData[i]/32767.0

    return signedData
