import os

def playPath (cl, robot, path):
    length = cl.problem.pathLength (robot, path)
    for i in range (101):
        l = i*length/100.
        cfg = cl.problem.configAtDistance (robot, path, l)
        cl.robot.setCurrentConfig (robot, cfg)

def isfloat (string) :
    try :
        float (string)
    except ValueError as exc :
        return False
    return True
    
def parseConfigInLog (pid, prefix):
    listConfig = []
    devel_dir = os.getenv ('DEVEL_DIR')
    with open (devel_dir + '/stable/var/log/hpp/journal.' + str(pid) + '.log',
               'r') as f:
        for line in f:
            if line[:len(prefix)] == prefix:
                configString = line [len(prefix):].strip(' ')
                config = map (float, filter (isfloat, configString.split (' ')))
                listConfig.append (config)
    return listConfig
