#!/usr/bin/python

# Script for restoring RoboCup 3D sim game from logfile
# Written by Patrick MacAlpine (patmac@cs.utexas.edu)
# Usage: ./sExprInterface.py <logFile> [host] [port]

import sys
import struct
import socket   #for sockets
import threading
import rospy
from threading import Lock
from robocup_msgs.msg import AgentState
from robocup_msgs.srv import SendJoints
from robocup_msgs.srv import InitAgent


class agentInterface:
  def __init__(self):
    # Agent internal data values
    self._agentTree = []
    self._initializeAgentValues()

    # Server internal data values
    self._serverTree = []
    self._initializeServerValues()

    self._serverTime = 0

    self.mutex = Lock()

  # Get tree from s-expression
  def _getTreeFromSExpr(self, sExpr):
    tree = []
    current = tree
    stack = [current]
    tokens = sExpr.split()

    symbol = ""
    for t in tokens:
      if symbol != "":
        current.append(symbol)
      symbol = ""
      for i in range(len(t)):
        if '(' == t[i]:
          if symbol != "":
            current.append(symbol)
          empty = []
          current.append(empty)
          stack.append(current)
          current = empty
          symbol = ""
        elif ')' == t[i]:
          if symbol != "":
            current.append(symbol)
          symbol = ""
          current = stack.pop()
        else:
          symbol = symbol + t[i]

    return tree

  # == Private agent methods ==

  def _initializeAgentValues(self):
    self._agentScene = None
    self._agentInit = None
    self._agentBeam = None
    self._agentJointRequests = []
    self._agentSay = None

  def _populateAgentValuesFromTree(self):
    self._initializeAgentValues()

    for i in self._agentTree:
      if i[0] == 'scene':
        self._agentScene = i[1]
      elif i[0] == 'init':
        self._agentInit = (i[1][1], i[2][1])
      elif i[0] == 'beam':
        self._agentBeam = (i[1], i[2], i[3])
      elif i[0].startswith("he") or i[0].startswith("lae") or i[0].startswith("lle") or i[0].startswith("rae") or i[0].startswith("rle"):
        self._agentJointRequests.append((i[0], i[1]))
      elif i[0] == 'say':
        self._agentSay = i[1]
      elif i[0] != 'syn':
        print "Unknown agent message type: " + i[0]

  # == Public agent methods ==

  def printAgentValues(self):
    print "Scene: " + str(self.getAgentScene())
    print "Init: " + str(self.getAgentInit())
    print "Beam: " + str(self.getAgentBeam())
    print "JointRequests: " + str(self.getAgentJointRequests())
    print "Say: " + str(self.getAgentSay())

  # Returns scene or None if scene doesn't exist
  def getAgentScene(self):
    return self._agentScene

  # Returns (unum, teamname) init values or None if init doesn't exist
  def getAgentInit(self):
    return self._agentInit

  # Returns (x, y, a) beam values or None if beam doesn't exist
  def getAgentBeam(self):
    return self._agentBeam

  # Returns [(name, value), ...] joint requests
  def getAgentJointRequests(self):
    return self._agentJointRequests

  # Returns say or None if say doesn't exist
  def getAgentSay(self):
    return self._agentSay

  # == Private server methods ==

  def _initializeServerValues(self):
    #self._serverTime = None
    #self._serverTime = 0
    #self._serverGameStateID = None
    self._serverGameStateID = 1
    #self._serverGameStateSide = None
    self._serverGameStateSide = 'left'
    #self._serverGameStateTime = None
    self._serverGameStateTime = 0
    #self._serverGameStatePlayMode = None
    self._serverGameStatePlayMode = 'BeforeKickOff'
    self._serverGameStateScoreLeft = None
    self._serverGameStateScoreRight = None
    self._serverGyro = None
    self._serverAccel = None
    #self._serverForceResistancePerceptors = []
    self._serverForceResistancePerceptors = [('lf', (0, 0, 0), (0, 0, 0)), ('rf', (0, 0, 0), (0, 0, 0))]
    self._serverHingeJoints = []
    self._serverHear = None
    self._serverSeenObjects = []
    self._serverSeenPlayers = []
    self._serverSeenLines = []
    self._serverMyPos = None
    self._serverMyOrien = None
    self._serverBallPos = None


  def _populateServerValuesFromTree(self):
    self._initializeServerValues()

    for i in self._serverTree:
      if i[0] == 'time':
        self._serverTime = i[1][1]
      elif i[0] == 'GS':
        self._serverGameState = []
        for j in i[1:]:
          if j[0] == 'unum':
            self._serverGameStateID = j[1]
          elif j[0] == 'team':
            self._serverGameStateSide = j[1]
          elif j[0] == 't':
            self._serverGameStateTime = j[1]
          elif j[0] == 'pm':
            self._serverGameStatePlayMode = j[1]
          elif j[0] == 'sl':
            self._serverGameStateScoreLeft = j[1]
          elif j[0] == 'sr':
            self._serverGameStateScoreRight = j[1]
          else:
            print "Unknown server game state type: " + j[0]
        self._agentInit = (i[1][1], i[2][1])
      elif i[0] == 'GYR':
        self._serverGyro = (i[1][1], i[2][1], i[2][2], i[2][3])
      elif i[0] == 'ACC':
        self._serverAccel = (i[1][1], i[2][1], i[2][2], i[2][3])
      elif i[0] == 'FRP':
        self._serverForceResistancePerceptors.append((i[1][1], (i[2][1], i[2][2], i[2][3]), (i[3][1], i[3][2], i[3][3])))
      elif i[0] == 'HJ':
        self._serverHingeJoints.append((i[1][1], i[2][1]))
      elif i[0] == 'hear':
        self._serverHear = i[1:]
      elif i[0] == 'See':
        for j in i[1:]:
          if j[0] == 'P':
            player = [j[1][1], j[2][1]]
            bodyParts = []
            for k in j[3:]:
              if k[0] == 'head' or k[0] == 'rlowerarm' or k[0] == 'llowerarm' or k[0] == 'rfoot' or k[0] == 'lfoot':
                bodyParts.append((k[0], k[1][1], k[1][2], k[1][3]))
              else:
                print "Unknown server player body part type: " + k[0]
            player.append(bodyParts)
            self._serverSeenPlayers.append(player)
          elif j[0] == 'L':
            self._serverSeenLines.append(((j[1][1], j[1][2], j[1][3]), (j[2][1], j[2][2], j[2][3])))
          elif j[0] == 'B' or j[0] == 'G1L' or j[0] == 'G1R' or j[0] == 'G2L' or j[0] == 'G2R' or j[0] == 'F1L' or j[0] == 'F1R' or j[0] == 'F2L' or j[0] == 'F2R':
            self._serverSeenObjects.append((j[0], j[1][1], j[1][2], j[1][3]))
          elif j[0] == 'mypos':
            self._serverMyPos = (j[1], j[2], j[3])
          elif j[0] == 'myorien':
            self._serverMyOrien = j[1]
          elif j[0] == 'ballpos':
            self._serverBallPos = (j[1], j[2], j[3])
          else:
            print "Unknown server vision type: " + j[0]
      else:
        print "Unknown server message type: " + i[0]

  # == Public server methods ==

  def makeTimeSExpr(self, time):
    return "(time (now " + str(time) + "))"

  # uNum and side are currently only given once right after an agent connects.  Side is either left or right
  def makeGameStateSExpr(self, gameTime, playMode, scoreLeft=None, scoreRight=None, uNum=None, side=None):
    ret = "(GS"
    if uNum != None:
      ret = ret + " (unum " + str(uNum) + ")"
    if side != None:
      ret = ret + " (team " + str(side) + ")"
    ret = ret + " (t " + str(gameTime) + ") (pm " + str(playMode) + ")"
    if scoreLeft != None:
      ret = ret + " (sl " + str(scoreLeft) + ")"
    if scoreRight != None:
      ret = ret + " (sr " + str(scoreRight) + ")"
    return ret + ")"

  def makeGyroSExpr(self, name, x, y, z):
    return "(GYR (n " + str(name) + ") (rt " + str(x) + " " + str(y) + " " + str(z) + "))"

  def makeAccelSExpr(self, name, x, y, z):
    return "(ACC (n " + str(name) + ") (a " + str(x) + " " + str(y) + " " + str(z) + "))"

  def makeHingeJointSExpr(self, name, ax):
    return "(HJ (n " + str(name) + ") (ax " + str(ax) + "))"

  # Names: lf (left foot), rf (right foot)
  def makeForceResistancePerceptorSExpr(self, name, px, py, pz, fx, fy, fz):
    return "(FRP (n " + str(name) + ") (c " + str(px) + " " + str(py) + " " + str(pz) + ") (f " + str(fx) + " " + str(fy) + " " + str(fz) + "))"

  def makeSeeSExpr(self, visionSExprs):
    return "(See " + visionSExprs + ")"

  # Must be put inside see message
  # Names: B (ball), G<1,2><L,R> (goal post <1,2> <left,right>), F<1,2><L,R> (corner flag <1,2> <left,right>)
  def makeSeenObjectSExpr(self, name, distance, theta, phi):
    return "(" + str(name) + " (pol " + str(distance) + " " + str(theta) + " " + str(phi) + "))"

  # Must be put inside see message
  def makeSeenLineSExpr(self, distance1, theta1, phi1, distance2, theta2, phi2):
    return "(L (pol " + str(distance1) + " " + str(theta1) + " " + str(phi1) + ") (pol " + str(distance2) + " " + str(theta2) + " " + str(phi2) + "))"

  # Must be put inside see message
  # bodyParts is a list of bodyparts where each bodypart is itself a list in the format (partName, distance, theta, phi).  Possible body parts are the following: head, rlowearam, llowerarm, rfoot, lfoot
  def makeSeenPlayerSExpr(self, team, id, bodyParts):
    ret = "(P (team " + str(team) + ") (id " + str(id) + ")"
    for p in bodyParts:
      ret = ret + " (" + str(p[0]) + " (pol " + str(p[1]) + " " + str(p[2]) + " " + str(p[3]) + "))"
    return ret + ")"

  # Must be put inside see message
  def makeGroundTruthMyPosSExpr(self, x, y, z):
    return "(mypos " + str(x) + " " + str(y) + " " + str(z) + ")"

  # Must be put inside see message
  def makeGroundTruthMyOrienSExpr(self, angle):
    return "(myorien " + str(angle) + ")"

  # Must be put inside see message
  def makeGroundTruthBallPosSExpr(self, x, y, z):
    return "(ballpos " + str(x) + " " + str(y) + " " + str(z) + ")"

  def makeHearSExpr(self, teamName, time, self_or_direction, message):
    if teamName == None:
      return "(hear " + str(time) + " " + str(self_or_direction) + " " + message + ")"
    return "(hear " + str(teamName) + " " + str(time) + " " + str(self_or_direction) + " " + message + ")"

  def makeSExprForAgent(self):
    msg = ""
    if self._serverTime != None:
      self._serverTime += 0.02
      msg = msg + self.makeTimeSExpr(self._serverTime)
    if self._serverGameStateTime != None and self._serverGameStatePlayMode != None:
      msg = msg + self.makeGameStateSExpr(self._serverGameStateTime, self._serverGameStatePlayMode, self._serverGameStateScoreLeft, self._serverGameStateScoreRight, self._serverGameStateID, self._serverGameStateSide)
    if self._serverGyro != None:
      msg = msg + self.makeGyroSExpr(self._serverGyro[0], self._serverGyro[1], self._serverGyro[2], self._serverGyro[3])
    if self._serverAccel != None:
      msg = msg + self.makeAccelSExpr(self._serverAccel[0], self._serverAccel[1], self._serverAccel[2], self._serverAccel[3])
    if len(self._serverSeenObjects) > 0 or len(self._serverSeenPlayers) > 0 or len(self._serverSeenLines) > 0 or self._serverMyPos != None or self._serverMyOrien != None or self._serverBallPos != None:
      visionMsgs = ""
      for i in self._serverSeenObjects:
        visionMsgs = visionMsgs + self.makeSeenObjectSExpr(i[0], i[1], i[2], i[3])
      for i in self._serverSeenPlayers:
        visionMsgs = visionMsgs + self.makeSeenPlayerSExpr(i[0], i[1], i[2])
      for i in self._serverSeenLines:
        visionMsgs = visionMsgs + self.makeSeenLineSExpr(i[0][0], i[0][1], i[0][2], i[1][0], i[1][1], i[1][2])
      if self._serverMyPos != None:
        visionMsgs = visionMsgs + self.makeGroundTruthMyPosSExpr(self._serverMyPos[0], self._serverMyPos[1], self._serverMyPos[2])
      if self._serverMyOrien != None:
        visionMsgs = visionMsgs + self.makeGroundTruthMyOrienSExpr(self._serverMyOrien)
      if self._serverBallPos != None:
        visionMsgs = visionMsgs + self.makeGroundTruthBallPosSExpr(self._serverBallPos[0], self._serverBallPos[1], self._serverBallPos[2])
      msg = msg + self.makeSeeSExpr(visionMsgs)
    for i in self._serverForceResistancePerceptors:
      msg = msg + self.makeForceResistancePerceptorSExpr(i[0], i[1][0], i[1][1], i[1][2], i[2][0], i[2][1], i[2][2])
    if self._serverHear != None:
      if len(self._serverHear) == 3:
        msg = msg + self.makeHearSExpr(None, self._serverHear[0], self._serverHear[1], self._serverHear[2])
      else:
        msg = msg + self.makeHearSExpr(self._serverHear[0], self._serverHear[1], self._serverHear[2], self._serverHear[3])
    for i in self._serverHingeJoints:
      msg = msg + self.makeHingeJointSExpr(i[0], i[1])
    return msg

  def callback(self, data):
    self.mutex.acquire()

    # Clear all values
    self._initializeServerValues()

    # Changing the time
    #self._serverTime += 0.02

    for i in range(len(data.joint_name)):
      # Populating the joints
      self._serverHingeJoints.append((data.joint_name[i], data.joint_angle_1[i]))

    # Set the gyro values
    #print self._serverGyro
    if self._serverGyro != None:
      self._serverGyro = ('torso', data.gyro_angular[0].x, data.gyro_angular[0].y, data.gyro_angular[0].z)

    if self._serverAccel != None:
      self._serverAccel = ('torso', data.gyro_linear[0].x, data.gyro_linear[0].y, data.gyro_linear[0].z)

    # Set the force sensor values

    # Set the perception
    for landmark in data.landmarks:
      self._serverSeenObjects.append((landmark.name, landmark.bearing.distance, landmark.bearing.angle1, landmark.bearing.angle2))

    for line in data.lines:
      self._serverSeenLines.append(((line.bearings[0].distance, line.bearings[0].angle1, line.bearings[0].angle2), (line.bearings[1].distance, line.bearings[1].angle1, line.bearings[1].angle2)))

    # Update the game state.
    self._serverGameStatePlayMode = data.game_state.play_mode
    self._serverGameStateScoreLeft = data.game_state.score_left
    self._serverGameStateScoreRight = data.game_state.score_right
    self._serverGameStateTime = data.game_state.time.to_sec()

    self.mutex.release()

  def initAgent(self, team, number):
    rospy.wait_for_service('/gameController/init_agent')
    try:
        init_agent_f = rospy.ServiceProxy('/gameController/init_agent', InitAgent)
        init_agent_f('/home/caguero/workspace/robocup_3d_simulation/models/teamA_1.sdf', team, number)
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e


  def sendJoints(self):
    rospy.wait_for_service('/teamA_1/send_joints')
    try:
        # if the list of joint commands is not 22, just return.
        if len(self._agentJointRequests) != 22:
          return

        send_joints_f = rospy.ServiceProxy('/teamA_1/send_joints', SendJoints)

        # Create a dictionary from the list
        toServer = {}
        #print 'agent joint request:'
        #print self._agentJointRequests
        for joint in self._agentJointRequests:
          #print 'joint:'
          #print joint
          toServer[joint[0]] = float(joint[1])

        newJoints = []
        newJoints.append(toServer['he1'])
        newJoints.append(toServer['he2'])
        newJoints.append(toServer['lle1'])
        newJoints.append(toServer['lle2'])
        newJoints.append(toServer['lle3'])
        newJoints.append(toServer['lle4'])
        newJoints.append(toServer['lle5'])
        newJoints.append(toServer['lle6'])
        newJoints.append(toServer['lae1'])
        newJoints.append(toServer['lae2'])
        newJoints.append(toServer['lae4'])
        newJoints.append(toServer['lae3'])
        newJoints.append(toServer['rle1'])
        newJoints.append(toServer['rle2'])
        newJoints.append(toServer['rle3'])
        newJoints.append(toServer['rle4'])
        newJoints.append(toServer['rle5'])
        newJoints.append(toServer['rle6'])
        newJoints.append(toServer['rae1'])
        newJoints.append(toServer['rae2'])
        newJoints.append(toServer['rae4'])
        newJoints.append(toServer['rae3'])

        send_joints_f(newJoints)
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e


  def run(self, s):
    rospy.Subscriber("/teamA_1/state", AgentState, self.callback)

    msgSize = s.recv(4)
    msgSize = struct.unpack("!L", msgSize)
    #print msgSize[0]
    msg = s.recv(msgSize[0])
    #print "From agent: " + msg
    self._agentTree = self._getTreeFromSExpr(msg)
    self._populateAgentValuesFromTree()
    #print self.agentTree
    #self.printAgentValues()

    #sserver = socket.socket()         # Create a sock et object
    #host = socket.gethostbyname("localhost")
    #print host
    #sserver.connect((host, 3100))

    #self.initAgent('team1', 1)

    while True:

      # Send joints to the server
      self.sendJoints()

      #msgToServer = struct.pack("!I", len(msg)) + msg
      #sserver.send(msgToServer)
      #msgSize = sserver.recv(4)
      #msgSize = struct.unpack("!L", msgSize)
      #print msgSize[0]
      #msgFromServer = sserver.recv(msgSize[0])
      #print "From server: " + msgFromServer
      #self._serverTree = self._getTreeFromSExpr(msgFromServer)
      #print self._serverTree
      #self._populateServerValuesFromTree()

      # Hardcoding the uniform number and side.


      msgForAgent = self.makeSExprForAgent()
      # print "Message for agent: " + msgForAgent
      msgToAgent = struct.pack("!I", len(msgForAgent)) + msgForAgent
      #msgToAgent = struct.pack("!I", len(msgFromServer)) + msgFromServer
      s.send(msgToAgent)
      msgSize = s.recv(4)
      msgSize = struct.unpack("!L", msgSize)
      #print msgSize[0]
      msg = s.recv(msgSize[0])
      #print "From agent: " + msg
      self._agentTree = self._getTreeFromSExpr(msg)
      self._populateAgentValuesFromTree()
      #print self.agentTree
      #print self.agentValues
      #print self.values
      #self.printAgentValues()




#sys.argv = [sys.argv[0], 'localhost', 3400]


if len(sys.argv) > 1 and sys.argv[1] == "--help":
  print "Usage: " + sys.argv[0] + " [host] [port]"
  sys.exit()


host = "localhost"
if len(sys.argv) > 1:
  host = sys.argv[1]

port = 3100
if len(sys.argv) > 2:
  port = int(sys.argv[2])


try:
    #create an AF_INET, STREAM socket (TCP)
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
except socket.error, msg:
    print 'Failed to create socket. Error code: ' + str(msg[0]) + ' , Error message : ' + msg[1]
    sys.exit();

serversocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

try:
    host = socket.gethostbyname( host )

except socket.gaierror:
    #could not resolve
    print 'Hostname could not be resolved. Exiting'
    sys.exit()

rospy.init_node('listener', anonymous=True)

#Connect to remote server
serversocket.bind((host, port))
serversocket.listen(22)

while True:
    #accept connections from outside
    (clientsocket, address) = serversocket.accept()
    #now do something with the clientsocket
    #in this case, we'll pretend this is a threaded server
    print 'Got connection from', address
    aI = agentInterface()
    #aI.run(clientsocket)
    t = threading.Thread(target = aI.run, args = (clientsocket,))
    t.start()
    #ct = client_thread(clientsocket)
    #ct.run()


s.close()
