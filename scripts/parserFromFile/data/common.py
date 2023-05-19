Start = ["start", "task"]

Name = ["Amelia", "Angel", "Ava", "Charlie", "Charlotte", "Hunter", "Max", "Mia",
        "Olivia", "Parker", "Sam", "Jack", "Noah", "Oliver", "Thomas", "William"]

Drink = ["Water", "Milk", "Coke", "Tonic", "Bubble Tea", "Ice Tea", "Wine", "Coffee", "Lemonade", "Iced tea", "Hot chocolate", "Juice", "Milkshake", "Tea", "Beer", "Soda",
         "Coffee", "Tea bag", "Tea", "Green tea", "Chocolate milk", "Hot chocolate", "Tomato juice", "Smoothie", "Coconut milk", "Orange juice", "Lemonade", "Fruit juice", "Cocoa"]

Locations = ["House Plant", "Coat Rack", "Sofa", "Couch Table", "TV", "Side Table", "Book Shelf", "Kitchen Shelf", "Pantry Pantry", "Dinner Table",
             "Kitchen Bin", "Fridge", "Washing Machine", "Sink", "Small Shelf", "Cupboard", "Big Shelf", "Bed", "Desk Snacks", "Show Rack", "Bin", "Office Shelf"]

Confirmation = ["Yes", "No"]

common_label = dict()
for label in ["Start", "Name", "Drink", "Locations", "Confirmation"]:
    common_label[label] = eval(label)

if __name__ == "__main__":
    print(common_label)
