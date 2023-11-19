#!/usr/bin/env python3
import generate_utils
import exceptions
import random

class Task_module:

    configuration = generate_utils.load_task_config()
    places = configuration["place_names"]
    language = configuration["languages"]
    question_tags = configuration["question_tags"]
    objects = configuration["objects"]

    def __init__(self, perception=False, speech=False, manipulation=False, navigation=False):
        """Initializer for the Task_module class

        Args:
            perception (bool): Enables or disables perception services
            speech (bool): Enables or disables speech services
            manipulation (bool): Enables or disables manipulation services
            navigation (bool): Enables or disables navigation services
        """
    ################### PERCEPTION SERVICES ###################


    def find_object(self,object_name:str, timeout=25)->bool:
        """
        Input:
        object_name: label of the object to look for options -> classes names depends of the actual model (see set_model service)
        timeout: time in seconds to look for the object while spinning ()
        Output: True if the object was found, False if not
        ----------
        Spins while looking for <object_name> for <timeout> seconds while spinning at 15 deg/s
        """
        if object_name not in self.objects:
            raise exceptions.InvalidObjectException(f"Object {object_name} not in the list of objects")
        return random.choice([True,False])

    def count_objects(self,object_name:str)->int:
        """
        Input: object_name, classes names depends of the actual model (see set_model service)
        Output: Number of objects seen when rotating 360
        ----------
        Spins 360 degrees and then returns the number of objects of <object_name> seen
        """
        if object_name not in self.objects:
            raise exceptions.InvalidObjectException(f"Object {object_name} not in the list of objects")
        return random.randint(0,10)

    def set_model(self,model_name:str)->bool:
        """
        Input: model name -> "default" || "objects" || "fruits"
        Output: True if the service was called correctly, False if not
        ----------
        Sets the model to use for the object recognition.
        classes by model:
        default: ["person","bench","backpack","handbag","suitcase","bottle","cup","fork","knife","spoon","bowl","chair","couch","bed","laptop"]
        objects: ["spam"(carne enlatada), "cleanser", "sugar", "jello"(gelatina roja), "mug", "tuna", "bowl", "tomato_soup", "footwear", "banana", "mustard", "coffee_grounds", "cheezit"]
        fruits: ["apple", "lemon", "orange", "peach", "pear", "plum","strawberry"]
        """

    ################### SPEECH SERVICES ###################

    def talk(self,text:str,language="English",wait=True,animated=False)->bool:
        """
        Input:
        text : text that robot will say
        language: English || Spanish
        wait(wait until the robot finishes talking)
        animates: gesture hands
        Output: True if everything ok || False if not
        ----------
        Allows the robot to say the input of the service.
        """
        if language not in self.language:
            raise exceptions.LanguageNotSupported(f"Language {language} not in the list of languages")
        return True

    def speech2text_srv(self, file_name="prueba",seconds=0,transcription=True)->bool:
        """
        Input:
        seconds: 0 for automatic stop || > 0 for seconds to record
        file_name: name of the file
        transcription: True || False
        Output: text that the robot heard
        ----------
        Allows the robot to save audio and saves it to a file.
        """
        return "prueba"

    def q_a_speech(self, tag:str)->str:
        """
        Input: tag in lowercase: options -> ("age", "name", "drink", "gender")
        Output: answer
        ----------
        Returns a specific answer for predefined questions.
        """
        if tag not in self.question_tags:
            raise exceptions.QuestionAnswerTagException(f"Tag {tag} not in the list of question tags")
        if tag =="age":
            return "1"
        else:
            return "prueba"
        
    def answer_question(self,question:str)->str:
        """
        Input: question
        Output: answer
        ----------
        Returns an answer for a question.
        """
        if question == "" or question is None:
            raise exceptions.EmptyQuestionException(f"Question recieved is empty")
        return "respuesta de prueba"

    ################### NAVIGATION SERVICES ###################

    def go_to_place(self,place_name:str, graph=1, wait=True)->bool:
        """
        Input:
        place_name: options -> ("bed","dishwasher","kitchen_table","dining_room","sink","desk","entrance","cleaning_stuff","bedside_table","shelf_bedroom","trashbin","pantry","refrigerator",cabinet","tv_stand","storage_rack","side_table","sofa","bookshelf")
        graph: 0 no graph || 1 graph
        wait: True (waits until the robot reaches) || False (doesn't wait)
        Output: True if the service was called correctly, False if not
        ----------
        Goes to place_name
        """
        if place_name not in self.places:
            raise exceptions.InvalidLocationException(f"Location {place_name} not in the list of locations")
        return True

    def robot_stop_srv(self)->bool:
        """
        Input: None
        Output: True if the service was called correctly, False if not
        ----------
        Stops the robot
        """

    def spin_srv(self, degrees:float):
        """
        Input: degrees
        Output: True if the service was called correctly, False if not
        ----------
        Spins the robot a number of degrees
        """
        if degrees < 0 or degrees > 360:
            raise exceptions.InvalidDegreesException(f"Degrees {degrees} not in the range [0,360]")

    def go_to_defined_angle_srv(self, degrees:float):
        """
        Input: degrees
        Output: True if the service was called correctly, False if not
        ----------
        Goes to defined angle
        """
        if degrees < 0 or degrees > 360:
            raise exceptions.InvalidDegreesException(f"Degrees {degrees} not in the range [0,360]")

    def follow_you(self)->bool:
        """
        Input:
        Output: True if the service was called correctly, False if not
        ----------
        Follows the person in front of the robot until the person touches the head of the robot,
        the service finishes by its own
        """

    def add_place(self,name:str,persist=0,edges=[])->bool:
        """
        Input: name, edges, persist
        Output: True if the service was called correctly, False if not
        ----------
        Adds a place to the graph
        """
        self.places.append(name)
        return True

    ############ MANIPULATION SERVICES ###############

    def grasp_object(self,object_name:str)->bool:
        """
        Input: object_name
        Output: True if the service was called correctly, False if not
        ----------
        Grasp the <object_name>
        """
        if object_name not in self.objects:
            raise exceptions.InvalidObjectException(f"Object {object_name} not in the list of objects")
        return True

    def leave_object(self,object_name:str)->bool:
        """
        Input: object_name
        Output: True if the service was called correctly, False if not
        ----------
        Leave the <object_name>
        """
        if object_name not in self.objects:
            raise exceptions.InvalidObjectException(f"Object {object_name} not in the list of objects")
        return True
