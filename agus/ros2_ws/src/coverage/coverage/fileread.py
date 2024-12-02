
class FileReader():
    def __init__(self, filename):
        try:
            self.f = open(filename, "r");
        except FileNotFoundError:
            print(f"The file {filename} was not found.")

    def readFromFile(self):
        pairs = []
        for line in self.f:
                # Strip any leading/trailing whitespaces and split by space (or any other delimiter)
                # assuming each line contains two real numbers separated by space
                numbers = line.strip().split()
                if len(numbers) == 2:
                    try:
                        # Convert the strings to float and store as a tuple in the array
                        num1 = float(numbers[0])
                        num2 = float(numbers[1])
                        pairs.append((num1, num2))
                    except ValueError:
                        print(f"Invalid number format in line: {line.strip()}")
                else:
                    print(f"Skipping invalid line: {line.strip()}")
        return pairs;