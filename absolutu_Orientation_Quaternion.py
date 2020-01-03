import numpy as np

def absoluti_Orirentation_Quaternion(camera,word,scale):
    [came_axis,came_num] = camera.shape
    [word_axis,word_num] = word.shape
    num = came_num
    c_mean = np.mean(camera, axis=1)
    w_mean = np.mean(word,axis=1)
    ##test
   ## print(c_mean)
    ##print(w_mean)
    came_nor = camera - c_mean.reshape(3,-1)
    word_nor = word - w_mean.reshape(3,-1)
    ###test
   # print(came_nor)
   # print(came_nor.shape)
   # print(word_nor)
    M = np.zeros((4,4))
    for i in range(num):
        a = np.vstack((np.zeros((1, 1)), (came_nor[:, i]).reshape(3, 1)))
        b = np.vstack((np.zeros((1, 1)), (word_nor[:, i]).reshape(3, 1)))
        Ma = np.array([[a[0],-a[1],-a[2],-a[3]],[a[1],a[0],a[3],-a[2]],[a[2],-a[3],a[0],a[1]],[a[3],a[2],-a[1],a[0]]])
        Mb = np.array([[b[0],-b[1],-b[2],-b[3]],[b[1],b[0],-b[3],b[2]],[b[2],b[3],b[0],-b[1]],[b[3],-b[2],b[1],b[0]]])
        Ma = Ma.reshape((4,4))
        Mb = Mb.reshape((4,4))
        ##test
        #print("{}th,{}".format(i,a))
        #print("{}th,{}".format(i,b))
        ###
        M = M + np.dot(Ma.T,Mb)
    ##test
    #print("M{}".format(M))
    ####
    [V, E] = np.linalg.eig(M)
    ind = np.argsort(V)
    E = E[:, ind]
    #print("E:{}".format(E))
    e = -E[:,3]
    M1 = np.array([[e[1-1],-e[2-1],-e[3-1],-e[4-1]],[e[2-1],e[1-1],e[4-1],-e[3-1]],[e[3-1],-e[4-1],e[1-1],e[2-1]],[e[4-1],e[3-1],-e[2-1],e[1-1]]])
    M2 = np.array([[e[1-1],-e[2-1],-e[3-1],-e[4-1]],[e[2-1],e[1-1],-e[4-1],e[3-1]],[e[3-1],e[4-1],e[1-1],-e[2-1]],[e[4-1],-e[3-1],e[2-1],e[1-1]]])
    #print(M1)
    #print(M2)
    R = np.dot(M1.T,M2)
    R = R[1:,1:]
    R = R.reshape((3,3))
    #print(R.shape)
    if scale!=0 :
        a=0;
        b=0;
        for i in  range(num):
            a = a + np.dot(np.dot(word_nor[:,i],R),came_nor[:,i])
            b = b + np.dot(word_nor[:,i].T,word_nor[:,i])
        s = b/a
    else:
        s = 1

    T = w_mean-s*np.dot(R,c_mean)

    return s,R,T


