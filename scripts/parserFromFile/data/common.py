Start = ["start", "task"]

Name = ["Adel", "Angel", "Axel", "Charlie", "Jane", "John", "Jules", "Morgan",
        "Paris", "Robin", "Simone"]

Drink = ["Water", "Milk", "Coke", "Tonic", "Bubble Tea", "Ice Tea", "Wine", "Coffee", "Lemonade", "Iced tea", "Hot chocolate", "Juice", "Milkshake", "Tea", "Beer", "Soda",
         "Coffee", "Tea bag", "Tea", "Green tea", "Chocolate milk", "Hot chocolate", "Tomato juice", "Smoothie", "Coconut milk", "Orange juice", "Lemonade", "Fruit juice", "Cocoa"]

Locations = ["Entrance", "Exit", "Bed", "Night table", "Desk", "Dinning table", "Couch", "End table", "Bookcase", "Cupboard", "Storage table", "Sink", "Counter", "Dishwasher"]


Confirmation = ["Yes", "No"]

common_label = dict()
for label in ["Start", "Name", "Drink", "Locations", "Confirmation"]:
    common_label[label] = eval(label)

if __name__ == "__main__":
    print(common_label)
