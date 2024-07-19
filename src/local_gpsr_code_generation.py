synonyms = {
    "fetch": ["bring", "get", "fetch", "grab", "retrieve"],
    "count": ["count", "number of", "how many"],
    "search": [
        "find",
        "search",
        "locate",
        "look for",
        "what",
        "heaviest",
        "lightest",
        "biggest",
        "smallest",
        "oldest",
        "youngest",
        "tallest",
        "shortest",
        "heavy",
        "light",
        "small",
        "big",
        "old",
        "young",
        "tall",
        "short",
    ],
    "follow": ["follow", "come after", "tail", "track"],
    "introduce": ["introduce", "presentation", "show", "present"],
    "describe": ["describe", "tell about", "detail", "explain"],
    "identify": ["identify", "who is", "recognize", "determine"],
    "navigate": ["go", "go to", "navigate", "move to", "head to"],
    "wait": ["wait", "pause", "hold on", "delay"],
    "inform": ["tell me", "inform", "advise", "notify"],
    "locate": ["where is", "find location of", "locate", "position of", "who"],
    "ask": ("ask", "who", "name"),
    "color": ("color", "red", "blue", "yellow"),
}

# Mapping related words to defined places
place_mappings = {
    "living_room": ["sofa", "tv", "living", "room", "television", "couch"],
    "hallway_cabinet": ["hallway_cabinet"],
    "desk": ["desk"],
    "shelf": ["shelf"],
    "coathanger": ["coathanger"],
    "exit": ["exit"],
    "tv_cable": ["tv_cable"],
    "lounge_chair": ["lounge_chair"],
    "lamp": ["lamp"],
    "couch": ["couch"],
    "coffee_table": ["coffee_table"],
    "trashcan": ["trashcan"],
    "kitchen_cabinet": ["kitchen_cabinet"],
    "dinner_table": ["dinner_table"],
    "dishwasher": ["dishwasher"],
    "kitchen_counter": ["kitchen_counter"],
    "hallway": ["hallway"],
    "office": ["office"],
    "kitchen": ["stove", "oven", "sink", "kitchen"],
    "dining_room": ["table", "dining", "chairs", "meal", "eat"],
    "bedroom": ["bed", "bedroom", "dresser", "pillow", "blanket"],
    "entrance": ["entrance", "door"],
    "refrigerator": ["kitchen"],
}

# Commonly ignored words that don't contribute to action determination
irrelevant_words = set(
    [
        "an",
        "a",
        "the",
        "and",
        "of",
        "to",
        "with",
        "at",
        "in",
        "him",
        "her",
        "he",
        "she",
        "it",
        "they",
        "right",
        "there",
        "is",
        "are",
        "was",
        "were",
        "be",
        "been",
        "have",
        "has",
        "had",
        "do",
        "does",
        "did",
        "can",
        "could",
        "will",
        "would",
        "shall",
        "should",
        "may",
        "might",
        "must",
        "about",
        "above",
        "across",
        "after",
        "against",
        "along",
        "among",
        "around",
        "as",
        "before",
        "behind",
        "below",
        "beneath",
        "beside",
        "between",
        "beyond",
        "but",
        "by",
        "despite",
        "down",
        "during",
        "except",
        "for",
        "from",
        "inside",
        "into",
        "like",
        "near",
        "off",
        "on",
        "onto",
        "out",
        "outside",
        "over",
        "past",
        "since",
        "through",
        "throughout",
        "till",
        "to",
        "toward",
        "under",
        "underneath",
        "until",
        "up",
        "upon",
        "without",
        "within",
        "yet",
    ]
)

action_keywords = {}


def init():
    for base_action, words in synonyms.items():
        for word in words:
            action_keywords[word] = base_action


def determine_place(words):
    # Initialize scores for each defined place
    score = {place: 0 for place in place_mappings.keys()}

    # Score each word found in the input against the place mappings
    for word in words:
        for place, keywords in place_mappings.items():
            if word in keywords:
                score[place] += 1

    # Determine the place with the highest score
    best_place = max(score, key=score.get)

    # Return the best place only if its score is greater than zero
    # This checks if any relevant words were actually found
    return best_place if score[best_place] > 0 else None


def robot_code_generator_aux(task):
    """
    Generates executable code for the robot based on the natural language task description.
    task: The requirement a person gave to the robot in Natural language
    """

    words = task.lower().split()
    filtered_words = [
        word for word in words if word not in irrelevant_words
    ]  # Filter out irrelevant words
    actions = []

    print(filtered_words)

    # Queue actions based on keywords using the updated mapping with synonyms
    for word in filtered_words:
        action = action_keywords.get(word)

    print(actions)

    # Initialize the task execution
    code = "print('Starting task execution')\n"
    code += "self.tm.talk('Processing your request', 'English', wait=False)\n"

    if (
        "kitchen"
        or "refrigerator"
        or "fridge"
        or "dish"
        or "washer"
        or "fruit"
        or "food"
        or "groceries" in filtered_words
    ):
        code += (
            "self.tm.talk('I will navigate to the kitchen.', 'English', wait=False)\n"
        )
        code += "self.tm.go_to_place('kitchen', wait=True)\n"
        code += (
            "self.tm.talk('I have arrived at the kitchen.', 'English', wait=False)\n"
        )

    elif "living_room" or "sofa" or "couch" or "tv" in filtered_words:
        code += "self.tm.talk('I will navigate to the living room.', 'English', wait=False)\n"
        code += "self.tm.go_to_place('living_room', wait=True)\n"
        code += "self.tm.talk('I have arrived at the living room.', 'English', wait=False)\n"

    elif "bedroom" or "lamp" or "bed" or "shelf" or "clothes" or "tv" in filtered_words:
        code += (
            "self.tm.talk('I will navigate to the bedroom.', 'English', wait=False)\n"
        )
        code += "self.tm.go_to_place('bedroom', wait=True)\n"
        code += (
            "self.tm.talk('I have arrived at the bedroom.', 'English', wait=False)\n"
        )

    elif "dinning room" or "table" or "chair" or "chairs" in filtered_words:
        code += "self.tm.talk('I will navigate to the dinning room.', 'English', wait=False)\n"
        code += "self.tm.go_to_place('dining_room', wait=True)\n"
        code += "self.tm.talk('I have arrived at the dining room.', 'English', wait=False)\n"

    # Enhanced action processing with all functionalities
    for i, word in enumerate(filtered_words):
        action = action_keywords.get(word)

        if not action:
            continue

        next_index = i + 1 if i + 1 < len(filtered_words) else i
        next_word = (
            filtered_words[next_index] if next_index < len(filtered_words) else ""
        )

        print(action)
        if action == "fetch":
            item = next_word
            code += f"if self.tm.look_for_object('{item}'):\n"
            code += f"    self.tm.talk('I have found {item}. Now I will bring it to you.', 'English', wait=False)\n"

            best_place = determine_place(filtered_words)
            if best_place and "go_to_place" not in code:
                code += f"self.tm.talk('I will now go to the {best_place}.', 'English', wait=False)\n"
                go_to_place_request = (
                    f"self.tm.go_to_place('{best_place}', wait=True)\n"
                )

                if go_to_place_request not in code:
                    code += go_to_place_request
                    code += f"self.tm.talk('I have arrived at the {best_place}.', 'English', wait=False)\n"

            code += f"self.tm.ask_for_object('{item}')\n"
            code += "self.tm.go_back()\n"
            code += f"self.tm.give_object('{item}')\n"
            code += f"self.tm.talk('Here is the object, thank you for using my services!', 'English', wait=False)\n"

        elif action == "count":
            item = next_word
            count = 0
            code += f"count = self.tm.count_objects('{item}')\n"
            code += "self.tm.go_back()\n"
            code += f"self.tm.talk('There are {count} {item}(s) present.', 'English', wait=False)\n"

        elif action == "find":
            code += f"if self.tm.look_for_object('person'):\n"
            code += f"    self.tm.talk('I have found you!', 'English', wait=False)\n"
            code += "else:\n"
            code += f"    self.tm.talk('I could not find the person.', 'English', wait=False)\n"
            code += "self.tm.go_back()\n"

        elif action == "follow":
            code += "self.tm.follow_you()\n"
            code += "self.tm.talk('Following you now. Please touch my head to stop.', 'English', wait=False)\n"
            code += "if self.tm.wait_for_head_touch():"
            code += "   self.tm.stop_following()\n"
            code += "   self.tm.talk('I have finished following you, I will now come back.', 'English', wait=False)\n"
            code += "self.tm.go_back()\n"

        elif action == "describe":
            code += "attributes = self.tm.get_person_description()\n"
            code += "self.tm.talk(f'The person is described as {attributes['gender']} and approximately {attributes['age']} years old.', 'English', wait=False)\n"

        elif action == "color":
            color = ""
            code += "color = self.tm.get_clothes_color()\n"
            code += "    self.tm.go_back()\n"
            code += f"self.tm.talk('The person is wearing {color} clothes.', 'English', wait=False)\n"

        elif action == "identify":
            gesture = ""
            code += "gesture = self.tm.get_person_gesture()"
            code += "    self.tm.go_back()\n"
            code += f"self.tm.talk('The person is {gesture}.', 'English', wait=False)\n"

        elif action == "wait_for_object":
            item = next_word
            code += f"if self.tm.wait_for_object(timeout=10):t\n"
            code += "     self.tm.go_back()\n"
            code += f"    self.tm.talk('I have spotted the {item}.', 'English', wait=False)\n"
            code += "else:\n"
            code += "     self.tm.go_back()\n"
            code += f"    self.tm.talk('I could not find the {item} after waiting.', 'English', wait=False)\n"

        # Handling object search with characteristics
        elif action == "search":

            if "heaviest" in filtered_words:
                characteristic = "heaviest"

            elif "lightest" in filtered_words:
                characteristic = "lightest"

            elif "biggest" in filtered_words:
                characteristic = "biggest"

            elif "smallest" in filtered_words:
                characteristic = "smallest"

            elif "oldest" in filtered_words:
                characteristic = "oldest"

            elif "youngest" in filtered_words:
                characteristic = "youngest"

            elif "tallest" in filtered_words:
                characteristic = "tallest"

            elif "shortest" in filtered_words:
                characteristic = "shortest"

            elif "heavy" in filtered_words:
                characteristic = "heavy"

            elif "light" in filtered_words:
                characteristic = "light"

            elif "small" in filtered_words:
                characteristic = "small"

            elif "big" in filtered_words:
                characteristic = "big"

            elif "red" in filtered_words:
                characteristic = "red"

            elif "blue" in filtered_words:
                characteristic = "blue"

            elif "green" in filtered_words:
                characteristic = "green"

            elif "yellow" in filtered_words:
                characteristic = "yellow"

            elif "round" in filtered_words:
                characteristic = "round"

            elif "square" in filtered_words:
                characteristic = "square"

            elif "dish" in filtered_words:
                characteristic = "dish"

            elif "fruit" in filtered_words:
                characteristic = "fruit"

            elif "food" in filtered_words:
                characteristic = "food"

            item_type = (
                filtered_words[next_index + 1]
                if next_index + 1 < len(filtered_words)
                else ""
            )
            code += f"item = self.tm.find_item_with_characteristic('{item_type}', '{characteristic}')\n"
            code += "self.tm.go_back()\n"
            code += f"self.tm.talk('I have found an item that matches the description of {characteristic}', 'English', wait=False)\n"

        # Handling navigation with dynamic re-routing
        elif action == "navigate":
            place = next_word
            code += f"self.tm.talk('I will now go to the {place}.', 'English', wait=False)\n"
            code += f"self.tm.go_to_place('{place}', wait=True)\n"
            code += f"self.tm.talk('I have arrived at the {place}', 'English', wait=False)\n"

        # Handling the retrieval and manipulation of an object
        elif action == "pick_object":
            item = next_word
            code += f"if self.tm.ask_for_object('{item}'):\n"
            code += f"    self.tm.talk('I have picked up the {item}.', 'English', wait=False)\n"
            code += "else:\n"
            code += f"    self.tm.talk('Unable to pick up the {item}.', 'English', wait=False)\n"
            code += "self.tm.go_back()\n"

        # Handling the introduction of the robot itself
        elif action == "introduce":
            code += "self.tm.talk('Hello, I am Pepper, your robotic assistant here to help.', 'English', wait=False)\n"

        # Handling commands to stop following a person
        elif action == "stop_following":
            code += "self.tm.stop_following()\n"
            code += "self.tm.talk('I have stopped following you as requested.', 'English', wait=False)\n"

        # Handling commands to dynamically adjust robot posture based on environmental feedback
        elif action == "adjust_posture":
            code += "self.tm.adjust_posture()\n"
            code += "self.tm.talk('Adjusting my posture for optimal performance.', 'English', wait=False)\n"

        elif action == "ask":
            request = next_word
            answer = ""
            code += f"answer = self.tm.q_a('Please tell me what is the {request}', 'English', wait=False)\n"
            code += "self.tm.go_back()\n"
            code += f"self.tm.talk('The name is {answer}', 'English', wait=False)\n"

        elif action == "go_back":
            code += "self.tm.go_back()\n"

    code += "print('Task execution completed')\n"
    return code


init()
request = "Go to the refrigerator and grab an orange"
print(robot_code_generator_aux(request))
