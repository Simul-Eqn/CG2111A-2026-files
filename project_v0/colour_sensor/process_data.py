import re

# Sample input string (you can also read this from a file)
data_red = """
Color: R=5110 Hz, G=1730 Hz, B=8320 Hz
c
Color: R=4970 Hz, G=1690 Hz, B=8090 Hz
c
Color: R=4870 Hz, G=1660 Hz, B=7970 Hz
c
Color: R=4950 Hz, G=1670 Hz, B=8030 Hz
c
Color: R=5340 Hz, G=1760 Hz, B=8550 Hz
"""
data_blue = '''Color: R=2620 Hz, G=3960 Hz, B=11570 Hz
c
Color: R=2700 Hz, G=4030 Hz, B=11760 Hz
c
Color: R=2710 Hz, G=4170 Hz, B=12230 Hz
c
Color: R=2740 Hz, G=4190 Hz, B=12230 Hz
c
Color: R=2800 Hz, G=4360 Hz, B=12710 Hz'''

data_green = '''Color: R=3290 Hz, G=3890 Hz, B=9590 Hz
c
Color: R=3320 Hz, G=3950 Hz, B=9720 Hz
c
Color: R=3340 Hz, G=3920 Hz, B=9660 Hz
c
Color: R=3300 Hz, G=3890 Hz, B=9610 Hz
c
Color: R=3310 Hz, G=3940 Hz, B=9700 Hz'''



# Regex pattern to extract R, G, B frequencies

pattern = r'Color: R=(\d+) Hz, G=(\d+) Hz, B=(\d+) Hz'

# Find all matches

def get_csv_string(data, c): 
    matches = re.findall(pattern, data)
    return '\n'.join(["{},{:d},{:d},{:d}".format(c, int(r), int(g), int(b)) for r, g, b in matches]) 


with open('values.csv', 'w') as f:
    f.write("COLOUR,R,G,B\n")
    f.write(get_csv_string(data_red, 'red'))
    f.write('\n')
    f.write(get_csv_string(data_blue, 'blue'))
    f.write('\n')
    f.write(get_csv_string(data_green, 'green'))
    f.write('\n') 



