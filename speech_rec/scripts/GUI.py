


# Defining colors
class bcolors:
	YELLOW = '\033[33m'
	BLUE = '\033[94m'
	CYAN = '\033[96m'
	GREEN = '\033[92m'
	RED = '\033[91m'
	END = '\033[0m'
	BOLD = '\033[1m'
	UNDERLINE = '\033[4m' 



title = R""" 
""" + bcolors.BOLD + """ """ + bcolors.CYAN + """
                                         .=*%%%%*=.               
                                       :#@@@@@@@@@@%:             
                                      -@@@@@@@@@@@@@@=            
                                      @@@@@@@@@@@@@@@@.           
                                     .@@@@@@@@@@@@@@@@:           
                                     .@@@@@@@@@@@@@@@@:           
                                     .@@@@@@@@@@@@@@@@:           
                                     .@@@@@@@@@@@@@@@@:           
                                 .-. .@@@@@@@@@@@@@@@@: .-.       
                                 *@@ .@@@@@@@@@@@@@@@@: @@#       
                                 +@@: %@@@@@@@@@@@@@@% :@@+       
                                  %@%..%@@@@@@@@@@@@%. %@@.       
                                  .%@@- =#@@@@@@@@%= -%@@:        
                                    +@@%=. :====:..=%@@+          
                                      =%@@@%#***#@@@%+.           
                                         -=+#@@%+=-               
                                            -@@=                  
                                            -@@=                  
                                        :===*@@*===:              
                                        *%%%%%%%%%%*  """ + bcolors.END + """

                                  """ + bcolors.BOLD +bcolors.GREEN +  """TIAGo VOCAL CONTROLLER 
"""+bcolors.END +bcolors.BOLD    

instruction = R"""
   Here is a list with the main possible voice commands, just say "Alexa" and the desired command:

   """ + bcolors.BOLD +bcolors.BLUE +  """ Base movements """ + bcolors.END + """                         """ + bcolors.BOLD +bcolors.BLUE +  """                         PlayMotions """ + bcolors.END + """                

   """ + bcolors.YELLOW + """    ============================================== """ + bcolors.END + """         """+ bcolors.YELLOW + """    ===================  """ + bcolors.END +"""
      |   Straight Right |    Go    |  Straight Left |             |  * hello        |
      |       Right      |          |      Left      |             |  * unfold       |
      |                  | Backward |                |             |  * maximum      |
   """ + bcolors.YELLOW + """    ============================================== """ + bcolors.END + """             |  * floor        |                                                                                 |  * shake        |
   """ + bcolors.BOLD +bcolors.BLUE +  """ Arm movements """ + bcolors.END + """                                                 |  * offer        |                                                                                 |  * surroundings |       
   """ + bcolors.YELLOW + """          ================================= """ + bcolors.END + """                    |  * tour         |
            |     Arm up     |    Elbow up    |                    |  * close        |
            |  ------------  |  ------------  |                    |  * half         |
            |     Arm down   |   Elbow down   |                    |  * gym          |
   """ + bcolors.YELLOW + """          ================================= """ + bcolors.END + """                    |  * home         |
""" + bcolors.YELLOW + """                                                                   ===================  """ + bcolors.END

endline= R"""
"""+ bcolors.GREEN +"""====================================================================================================\n"""+bcolors.END

