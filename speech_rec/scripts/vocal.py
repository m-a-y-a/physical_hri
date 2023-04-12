
###### Vocal Commands ########

'''

A special function is associated with each voice command that sets 
two integers (mode, type) recognizable by the other nodes.

'''

#### Mode 1 ####
def hello(word):
    print('Hello from TIAGo')
    mode = 1
    kind = 1
    name = 'hello'
    info = 'say hello with the arm'
    return mode, kind,name,info

def unfold(word):
    print('unfold')
    mode = 1
    kind = 2
    name = 'unfold'
    info = 'unfolds the arm'
    return mode, kind,name,info

def maximum(word):
    print('maximum')
    mode = 1
    kind = 3
    name = 'maximum'
    info = 'reaches the maximum'
    return mode, kind,name,info

def floor(word):
    print('floor')
    mode = 1
    kind = 4
    name = 'hello'
    info = 'reaches the floor'
    return mode, kind,name,info

def shake(word):
    print('shake')
    mode = 1
    kind = 5
    name = 'shake'
    info = 'shake the hand'
    return mode, kind,name,info

def offer(word):
    print('offer')
    mode = 1
    kind = 6
    name = 'offer'
    info = 'offer'
    return mode, kind,name,info

def surroundings(word):
    print('surroundings')
    mode = 1
    kind = 7
    name = 'surroundings'
    info = 'inspect the surroundings with the head'
    return mode, kind,name,info

def tour(word):
    print('tour')
    mode = 1
    kind = 8
    name = 'tour'
    info = 'makes a head-tour'
    return mode, kind,name,info

def close(word):
    print('close')
    mode = 1
    kind = 9
    name = 'close'
    info = 'closes the arm'
    return mode, kind,name,info

def half(word):
    print('half')
    mode = 1
    kind = 10
    name = 'half'
    info = 'semi-closes the arm'
    return mode, kind,name,info

def gym(word):
    print('gym')
    mode = 1
    kind = 11
    name = 'gym'
    info = 'does weights with the arm'
    return mode, kind,name,info

def home(word):
    print('home')
    mode = 1
    kind = 12
    name = 'hello'
    info = 'returns in home configuration'
    return mode, kind,name,info

#### Mode 2 ####
def go(word):
    print('OKKK, Lets gooo')
    mode = 2
    kind = 1
    name = 'go'
    info = 'goes straight-forward'
    return mode, kind,name,info

def accelerate(word):
    print('accelerate')
    mode = 2
    kind = 6
    name = 'accelerate'
    info = 'accelerates'
    return mode, kind,name,info

def decelerate(word):
    print('decelerate')
    mode = 2
    kind = 7
    name = 'decelerate'
    info = 'decelerates'
    return mode, kind,name,info

def reset(word):
    print('reset')
    mode = 2
    kind = 8
    name = 'reset'
    info = 'resets the velocity'
    return mode, kind,name,info

def left(word):
    print('left')
    mode = 2
    if 'straight' in word:
            kind = 5
            name = 'straight left'
            info = 'goes straight-left'
    else:
            kind = 2
            name = 'left'
            info = 'turns left'
    return mode, kind,name,info

def right(word):
    print('right')
    mode = 2
    if 'straight' in word:
            kind = 4
            name = 'straight right'
            info = 'goes straight-right'
    else:
            kind = 3
            name = 'right'
            info = 'turns right'
    return mode, kind,name,info


def backward(word):
    print('BACKWARDS')
    mode = 2
    kind = 9
    name = 'backward'
    info = 'goes backwards'
    return mode, kind,name,info

def stop(word):
    print('STOP')
    mode = 2
    kind = -1
    name = 'stop'
    info = 'stops the motion'
    return mode, kind,name,info

#### Mode 3 ####
def arm(word):
    print('Arm: '+word)

    mode=3
    if 'up' in word:
            kind = 1
            name = 'arm up'
            info = 'increases the shoulder joint'
    elif 'down' in word:
            kind = 2
            name = 'arm down'
            info = 'decreases the shoulder joint'
    else:
        kind=-1
        name = ''
        info = ''    
    return mode, kind,name,info

def elbow(word):

    mode=3
    if 'up' in word:
            kind = 3
            name = 'elbow up'
            info = 'increases the elbow joint'
    elif 'down' in word:
            kind = 4
            name = 'elbow down'
            info = 'decreases the elbow joint'
    else:
        kind=-1 
        name = ''
        info = ''   
    return mode, kind,name,info


#### Default ####
def default(word):
    print ("Command not recognized, try again..")
    mode = -1
    kind = -1
    name = ''
    info = ''
    return mode, kind,name,info

# Dictionary that contain all the possible vocal commands

switcher = {

    'hello': hello,

    'unfold': unfold,

    'maximum': maximum,

    'floor': floor,

    'shake': shake,

    'offer': offer,

    'surroundings': surroundings,

    'tour': tour,

    'close': close,

    'half': half,

    'gym': gym,

    'home': home,

    'go': go,

    'left' : left,

    'right': right,

    'backward': backward,

    'accelerate': accelerate,

    'decelerate': decelerate,

    'reset': reset,

    'arm': arm,

    'elbow': elbow,

    'stop': stop
}


def word(word):

    """

    Function to call the proper action according to the vocal input

    """
    command=""
    # Browsing in the dictionary to know if the vocal input is right 
    # or not 

    for key in switcher:
        if key in word:
            command=key

    # getting the riht function to call from the dictionary 
    function = switcher.get(command,default)
    return function(word)

