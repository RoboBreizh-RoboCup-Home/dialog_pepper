#================================================================================================================#
#                                                    OBJECTS                                                     #
#================================================================================================================#
def objects_list():
    
    objects_a = [   'kleenex', 'whiteboard cleaner', 'cup', 'snack', 'cereals bar', 'cookie', 'book', 'pen', 'notebook', 'laptop', 'tablet', 'charger',
                    'pencil', 'peanut', 'biscuit', 'candy', 'chocolate bar', 'chewing gum', 'chocolate egg', 'chocolate tablet', 'donuts', 'cake', 'pie',
                    'peach', 'strawberry', 'blueberry', 'blackberry', 'burger', 'lemon', 'lemon', 'banana', 'watermelon', 'pepper', 'pear', 'pizza',
                    'yogurt', 'drink', 'beer', 'coke', 'sprite', 'sake', 'toothpaste', 'cream', 'lotion', 'dryer', 'comb', 'towel', 'shampoo', 'soap',
                    'cloth', 'sponge', 'toothbrush', 'container', 'glass', 'can', 'bottle', 'fork', 'knife', 'bowl',
                    'tray', 'plate', 'newspaper', 'magazine']

    objects_an = [  'almond', 'onion', 'orange', 'apple']

    objects_the = [ 'cookies', 'almonds', 'book', 'pen', 'notebook', 'laptop', 'tablet', 'charger', 'pencil', 'chips', 'senbei', 'pringles',
                    'peanuts', 'biscuits', 'crackers', 'candies', 'chocolate bar', 'manju', 'mints', 'chewing gums', 'chocolate egg', 'chocolate tablet',
                    'donuts', 'cake', 'pie', 'food', 'peach', 'strawberries', 'grapes', 'blueberries', 'blackberries', 'salt', 'sugar', 'bread', 'cheese',
                    'ham', 'burger', 'lemon', 'onion', 'lemons', 'apples', 'onions', 'orange', 'oranges', 'peaches', 'banana', 'bananas', 'noodles',
                    'apple', 'paprika', 'watermelon', 'sushi', 'pepper', 'pear', 'pizza', 'yogurt', 'drink', 'milk', 'juice', 'coffee', 'hot chocolate',
                    'whisky', 'rum', 'vodka', 'cider', 'lemonade', 'tea', 'water', 'beer', 'coke', 'sprite', 'wine', 'sake', 'toiletries', 'toothpaste',
                    'cream', 'lotion', 'dryer', 'comb', 'towel', 'shampoo', 'soap', 'cloth', 'sponge', 'toilet paper', 'toothbrush', 'container', 'containers',
                    'glass', 'can', 'bottle', 'fork', 'knife', 'bowl', 'tray', 'plate', 'newspaper', 'magazine', 'rice','kleenex', 'whiteboard cleaner', 'cup']

    objects_some = ['snacks', 'cookies', 'almonds', 'books', 'pens', 'chips', 'pringles', 'magazines', 'newspapers', 'peanuts', 'biscuits',
                    'crackers', 'candies', 'mints', 'chewing gums', 'donuts', 'cake', 'pie', 'food', 'strawberries', 'grapes', 'blueberries',
                    'blackberries', 'salt', 'sugar', 'bread', 'cheese', 'ham', 'lemons', 'apples', 'onions', 'oranges', 'peaches', 'bananas',
                    'noodles', 'paprika', 'watermelon', 'sushi', 'pepper', 'pizza', 'yogurt', 'drink', 'milk', 'juice', 'coffee', 'hot chocolate',
                    'whisky', 'rum', 'vodka', 'cider', 'lemonade', 'tea', 'water', 'beer', 'coke', 'sprite', 'wine', 'sake', 'toilet paper',
                    'containers', 'glasses', 'cans', 'bottles', 'forks', 'knives', 'bowls', 'trays', 'plates', 'lemon', 'rice', 'cups']

    objects_a_piece_of = ['apple', 'lemon', 'cake', 'pie', 'bread', 'cheese', 'ham', 'watermelon', 'sushi', 'pizza']

    objects_a_cup_of = ['juice', 'rice', 'milk', 'coffee', 'hot chocolate', 'cider', 'lemonade', 'tea', 'water', 'beer']

    objects_a_can_of = ['juice', 'kleenex', 'red bull', 'cider', 'iced tea', 'beer', 'coke', 'sprite']

    objects_a_glass_of = [  'milk', 'juice', 'coffee', 'hot chocolate', 'whisky', 'rum', 'vodka', 'cider', 'lemonade', 'tea', 'water', 'beer',
                            'coke', 'sprite', 'wine', 'sake']

    objects_a_bottle_of = [ 'kleenex', 'milk', 'juice', 'whisky', 'rum', 'vodka', 'cider', 'lemonade',
                            'iced tea', 'water', 'beer', 'coke', 'sprite', 'wine','sake']

    objects = {}
    objects["a"] = list(set(objects_a))
    objects["an"] = list(set(objects_an))
    objects["the"] = list(set(objects_the))
    objects["some"] = list(set(objects_some))
    objects["a piece of"] = list(set(objects_a_piece_of))
    objects["a cup of"] = list(set(objects_a_cup_of))
    objects["a can of"] = list(set(objects_a_can_of))
    objects["a glass of"] = list(set(objects_a_glass_of))
    objects["a bottle of"] = list(set(objects_a_bottle_of))

    return objects

#////////////////////////////////////////////////////////////////////////////////////////////////////////////////#

#================================================================================================================#
#                                                  DESTINATIONS                                                  #
#================================================================================================================#
def destination_list():

    destinations_on = [ 'nightstand', 'bookshelf', 'coffee table', 'side table', 'kitchen table', 'kitchen cabinet',
                        'tv stand', 'sofa', 'couch', 'bedroom chair', 'kitchen chair', 'living room table', 'center table',
                        'drawer', 'desk', 'cupboard', 'side shelf', 'bookcase', 'dining table', 'fridge', 'counter',
                        'cabinet', 'table', 'bedchamber', 'chair', 'dryer', 'oven', 'rocking chair', 'stove', 'television', 'bed', 'dressing table',
                        'bench', 'futon', 'beanbag', 'stool', 'sideboard', 'washing machine', 'dishwasher']

    destinations_in = [ 'wardrobe', 'nightstand', 'bookshelf', 'dining room', 'bedroom', 'closet', 'living room', 'bar', 'office',
                        'drawer', 'kitchen', 'cupboard', 'side shelf', 'refrigerator', 'corridor', 'cabinet', 'bathroom', 'toilet', 'hall', 'hallway',
                        'master bedroom', 'dormitory room', 'bedchamber', 'cellar', 'den', 'garage', 'playroom', 'porch', 'staircase', 'sunroom', 'music room',
                        'prayer room', 'utility room', 'shed', 'basement', 'workshop', 'ballroom', 'box room', 'conservatory', 'drawing room',
                        'games room', 'larder', 'library', 'parlour', 'guestroom', 'crib', 'shower']

    destinations_at = [ 'wardrobe', 'nightstand', 'bookshelf', 'coffee table', 'side table', 'kitchen table', 'kitchen cabinet',
                        'bed', 'bedside', 'closet', 'tv stand', 'sofa', 'couch', 'bedroom chair', 'kitchen chair',
                        'living room table', 'center table', 'bar', 'drawer', 'desk', 'cupboard', 'sink', 'side shelf',
                        'bookcase', 'dining table', 'refrigerator', 'counter', 'door', 'cabinet', 'table', 'master bedroom', 'dormitory room',
                        'bedchamber', 'chair', 'dryer', 'entrance', 'garden', 'oven', 'rocking chair', 'room', 'stove', 'television', 'washer',
                        'cellar', 'den', 'laundry', 'pantry', 'patio', 'balcony', 'lamp', 'window', 'lawn', 'cloakroom', 'telephone', 'dressing table',
                        'bench', 'futon', 'radiator', 'washing machine', 'dishwasher']

    destinations = [["on", "in", "at", "to", "from"], list(set(destinations_on + destinations_in + destinations_at))]

    return destinations

#////////////////////////////////////////////////////////////////////////////////////////////////////////////////#

#================================================================================================================#
#                                                    NAMES                                                       #
#================================================================================================================#
def name_list():

    names_female = ['hanna', 'barbara', 'samantha', 'erika', 'sophie', 'jackie', 'skyler', 'jane', 'olivia', 'emily', 'amelia', 'lily',
                    'grace', 'ella', 'scarlett', 'isabelle', 'charlotte', 'daisy', 'sienna', 'chloe', 'alice', 'lucy', 'florence', 'rosie',
                    'amelie', 'eleanor', 'emilia', 'amber', 'ivy', 'brooke', 'summer', 'emma', 'rose', 'martha', 'faith', 'amy', 'katie',
                    'madison', 'sarah', 'zoe', 'paige', 'mia', 'emily', 'sophia', 'abigail', 'isabella', 'ava', "jeannie", 'julie', 'jessica',
                    'jennifer', 'mary']
    names_male = [  'ken', 'erik', 'samuel', 'skyler', 'brian', 'thomas', 'edward', 'michael', 'charlie', 'alex', 'john', 'james', 'oscar',
                    'peter', 'oliver', 'jack', 'harry', 'henry', 'jacob', 'thomas', 'william', 'will', 'joshua', 'josh', 'noah', 'ethan', 'joseph',
                    'samuel', 'daniel', 'max', 'logan', 'isaac', 'dylan', 'freddie', 'tyler', 'harrison', 'adam', 'theo', 'arthur', 'toby', 'luke',
                    'lewis', 'matthew', 'harvey', 'ryan', 'tommy', 'michael', 'nathan', 'blake', 'charles', 'connor', 'jamie', 'elliot', 'louis',
                    'aaron', 'evan', 'seth', 'liam', 'mason', 'alexander', 'madison', 'valentin', 'trajano', 'simon', 'ignacio', 'francis', 'robin']

    return list(set(names_male + names_female))

#////////////////////////////////////////////////////////////////////////////////////////////////////////////////#

#================================================================================================================#
#                                               ACTION'S SYNONYMS                                                #
#================================================================================================================#
def sinonyms_list():

    synonyms_go = ["go", "navigate", "travel", "move", "drive", "come", "walk", "reach"]
    synonyms_find = ["find", "meet", "look", "locate", "face", "contact", "greet"]
    synonyms_follow = ["follow"]
    synonyms_take = ["take", "grasp", "pick", "bring", "deliver", "get", "put", "give", "distribute", "guide", "lead", "escort", "accompany", "provide"]
    synonyms_place = ["place", "throw", "dump"]
    synonyms_say = ["say", "tell", "mention", "answer", "ask", "describe"]
    
    return synonyms_go, synonyms_find, synonyms_follow, synonyms_take, synonyms_place, synonyms_say