import os

word_size, num_words  = os.environ["RAM_CONFIG"].split("_")

# Data word size
word_size = int(word_size)
# Number of words in the memory
num_words = int(num_words)

# Technology to use in $OPENRAM_TECH
tech_name = "freepdk45"

num_rw_ports = 0
num_r_ports = 1
num_w_ports = 1

# You can use the technology nominal corner only
nominal_corner_only = True
# Or you can specify particular corners
# Process corners to characterize
# process_corners = ["SS", "TT", "FF"]
# Voltage corners to characterize
# supply_voltages = [ 3.0, 3.3, 3.5 ]
# Temperature corners to characterize
# temperatures = [ 0, 25 100]

# Output directory for the results
output_path = "build"
# Output file base name
output_name = "sram_{0}_{1}_{2}_analytical".format(word_size,num_words,tech_name)

# Disable analytical models for full characterization (WARNING: slow!)
# analytical_delay = False
