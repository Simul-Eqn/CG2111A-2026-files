import csv
import re

input_files = ["dataset4.txt", 'dataset5.txt', 'dataset6.txt', 'redorange2.txt', 'redorange1.txt']
output_file = "values.csv"

# Regular expression to match lines with r,g,b,colour
pattern = re.compile(r"([\d.]+),([\d.]+),([\d.]+),([A-Z_]+),\d+")


with open(output_file, "w", newline="") as outfile:
    writer = csv.writer(outfile)
    writer.writerow(["R", "G", "B", "COLOUR"])
    
    for input_file in input_files:
        with open(input_file, "r") as infile: 
            
            for line in infile:
                match = pattern.search(line)
                if match:
                    r, g, b, colour = match.groups()
                    writer.writerow([r, g, b, colour])

print(f"Extracted data saved to {output_file}")
