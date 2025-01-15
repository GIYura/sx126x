#######################
# Directory shortcut
#######################

#######################
# Output directories
#######################

# Check if .config file is present and print error if not
ifeq (, $(wildcard .config))
    $(error ERROR: '.config' is missing. Please create '.config' file based on default.config as template.)
endif

-include .config

#######################
# CFLAGS
#######################
