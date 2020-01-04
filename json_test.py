import json

parameters = {}
parameters['s']=1.02
parameters['R']=[[1,2,3],[4,5,6],[7,5,7]]
parameters['T']=[[7],[8],[9]]

with open('parameters.json','w') as f:
    json.dump(parameters,f)

with open('parameters.json','r') as f:
    p = json.load(f)
    print(p)
