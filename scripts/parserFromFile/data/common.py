Start = ["start", "task"]

Name = ["Adel", "Angel", "Axel", "Charlie", "Jane", "John", "Jules", "Morgan",
        "Paris", "Robin", "Simone"]

Drink = [ "Cola","Iced tea","Ice Tea","Juice pack","Milk","Orange juice","red wine","tropical juice", "Water", "Coke","Tonic", "Bubble Tea",  "Wine", "Coffee", "Lemonade",  "Hot chocolate",  "Milkshake", "Tea", "Beer", "Soda",
         "Coffee", "Tea bag", "Tea", "Green tea", "Chocolate milk", "Hot chocolate", "Tomato juice", "Smoothie", "Coconut milk",  "Lemonade", "Fruit juice", "Cocoa"]

Locations = ["Entrance", "Exit", "Bed", "Night table", "Desk", "Dinning table", "Couch", "End table", "Bookcase", "Cupboard", "Storage table", "Sink", "Counter", "Dishwasher"]


Confirmation = ["Yes", "No"]

common_label = dict()
for label in ["Start", "Name", "Drink", "Locations", "Confirmation"]:
    common_label[label] = eval(label)

if __name__ == "__main__":
    print(common_label)
