import numpy as np
import pandas as pd
import xlrd
import absolutu_Orientation_Quaternion as aoq

file_path = r'test_data.xlsx'
data = xlrd.open_workbook(file_path)
table = data.sheet_by_name('Sheet1')
# get position in the camera coordination
c_x = table.col_values((0))[1:]
c_y = table.col_values((1))[1:]
c_z = table.col_values((2))[1:]
c_x = np.array(c_x)
c_y = np.array(c_y)
c_z = np.array(c_z)
camera_cor = np.vstack((c_x,c_y,c_z))
# get position in the word coordination
w_x = table.col_values((3))[1:]
w_y = table.col_values((4))[1:]
w_z = table.col_values((5))[1:]
w_x = np.array(w_x)
w_y = np.array(w_y)
w_z = np.array(w_z)
word_cor = np.vstack((w_x,w_y,w_z))
# set the initial doScale = 1
doScale = 1


######
s,R,T = aoq.absoluti_Orirentation_Quaternion(camera_cor,word_cor,doScale)
print(s)
print(R)
print(T)