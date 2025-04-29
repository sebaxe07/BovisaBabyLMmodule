class Colors:
    # ANSI color codes
    BLACK = '\033[30m'
    RED = '\033[31m'
    GREEN = '\033[32m'
    YELLOW = '\033[33m'
    BLUE = '\033[34m'
    MAGENTA = '\033[35m'
    CYAN = '\033[36m'
    WHITE = '\033[37m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'
    RESET = '\033[0m'

# Module-specific colors
MODULE_COLORS = {
    'camera': Colors.BLUE,
    'main': Colors.GREEN,
    'lidar': Colors.CYAN,
    'arduino': Colors.GREEN,
    'controller': Colors.YELLOW,
    'flask': Colors.MAGENTA,
    'gps': Colors.MAGENTA,
    'error': Colors.RED,
    'warning': Colors.YELLOW,
    'info': Colors.WHITE,
    'debug': Colors.BLUE
}

def colored_print(module, message, level="info"):
    """Print with color based on module and log level"""
    module_color = MODULE_COLORS.get(module.lower(), Colors.WHITE)
    level_color = MODULE_COLORS.get(level.lower(), Colors.WHITE)
    
    print(f"{module_color}[{module}]{Colors.RESET} {level_color}{message}{Colors.RESET}")

# Example logging functions for convenience
def log_error(module, message):
    colored_print(module, message, "error")
    
def log_info(module, message):
    colored_print(module, message, "info")
    
def log_debug(module, message):
    colored_print(module, message, "debug")