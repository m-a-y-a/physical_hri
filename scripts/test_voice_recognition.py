import speech_recognition as sr


def get_voice_cmd():
    try:
        # rospy.loginfo("in get_voice_cmd")
        # self.say("I am listening")
        print("I am listening")
        listener = sr.Recognizer()

        with sr.Microphone() as src:

            listener.adjust_for_ambient_noise(src)
            
            # rospy.loginfo("trying to listen")
            print("trying to listen")
            # audio = self.listener.listen(src, timeout=10)
            audio = listener.listen(src)
            # rospy.loginfo("trying google recognition")
            print("trying google recognition")
            cmd = listener.recognize_google(audio, language = "en-EN")
            cmd = cmd.lower()  
            print(cmd)    
            # send_cmd(cmd)  
            return cmd
    except Exception as e:
        # rospy.logerr("Exception %s occurred", str(e))
        print("error occured: {0}".format(e))

def send_cmd():
    # rospy.loginfo("in send_cmd")
    while True:
        cmd = get_voice_cmd()
        # rospy.logdebug("Voice detected %s", cmd)
        try:
            list_cmd = cmd.split(" ")
            print(list_cmd)
            if (cmd == "hello"):
                self.say(cmd)
            elif(cmd == "good job"):
                self.say("thank you")
            elif (cmd == "thank you"):
                self.say("you are welcome")
            elif "water" in list_cmd:
                # self.say("getting the water bottle")
                print("getting the water bottle")
            elif "nuts" in list_cmd:
                # self.say("getting the mixed nuts")
                print("getting the mixed nuts")
            elif "medicine" in list_cmd or "pill" in list_cmd:
                # self.say("getting the pill bottle")
                print("getting the pill bottle")
            elif "oats" in list_cmd:
                # self.say("getting the oats")
                print("getting the oats")
            elif "done" in list_cmd:
                print("done")
                return False
        except Exception as e:
            continue




if __name__ == "__main__":
    send_cmd()