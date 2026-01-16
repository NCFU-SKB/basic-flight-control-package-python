import logging
from   datetime import datetime
from   pathlib import Path

class Colors:
    """ANSI color codes for console output."""
    RESET = '\033[0m'
    BOLD = '\033[1m'
    
    # Regular colors
    BLACK = '\033[30m'
    RED = '\033[31m'
    GREEN = '\033[32m'
    YELLOW = '\033[33m'
    BLUE = '\033[34m'
    MAGENTA = '\033[35m'
    CYAN = '\033[36m'
    WHITE = '\033[37m'
    
    # Bright colors
    BRIGHT_BLACK = '\033[90m'
    BRIGHT_RED = '\033[91m'
    BRIGHT_GREEN = '\033[92m'
    BRIGHT_YELLOW = '\033[93m'
    BRIGHT_BLUE = '\033[94m'
    BRIGHT_MAGENTA = '\033[95m'
    BRIGHT_CYAN = '\033[96m'
    BRIGHT_WHITE = '\033[97m'


class CustomDroneLogger:
    """Custom logger wrapper with file logging and colorized console output."""
    
    def __init__(self, ros_logger, drone_name: str = "drone"):
        self.ros_logger = ros_logger
        self.drone_name = drone_name
        self.file_logger = self._setup_file_logger()
        
        # Color mapping for different log levels
        self.level_colors = {
            'DEBUG': Colors.BRIGHT_BLACK,
            'INFO': Colors.GREEN,
            'WARNING': Colors.YELLOW,
            'ERROR': Colors.RED,
            'CRITICAL': Colors.BRIGHT_RED,
            'FATAL': Colors.BRIGHT_RED,
        }
        
    def _setup_file_logger(self) -> logging.Logger:
        """Setup dedicated file logger with proper formatting."""
        # Find flight_control package directory
        current_path = Path(__file__).resolve()
        
        # Look for src/flight_control in the path hierarchy
        package_dir = None
        for parent in current_path.parents:
            potential_package = parent / "src" / "flight_control"
            if potential_package.exists() and potential_package.is_dir():
                package_dir = potential_package
                break
        
        # Fallback: try to find it differently
        if package_dir is None:
            for parent in current_path.parents:
                if parent.name == "flight_control" and (parent.parent.name == "src" or "flight_control" in str(parent)):
                    package_dir = parent
                    break
        
        # Final fallback to current directory
        if package_dir is None:
            package_dir = Path.cwd()
            print(f"[WARN] Could not find flight_control package, using: {package_dir}")
        
        # Create logs directory in package
        logs_dir = package_dir / "logs"
        logs_dir.mkdir(exist_ok=True)
        
        print(f"[DEBUG] Creating logs in: {logs_dir}")
        
        # Count existing log files
        try:
            existing_logs = list(logs_dir.glob("*.log"))
            log_number = len(existing_logs) + 1
        except Exception as e:
            print(f"[ERROR] Failed to count existing logs: {e}")
            log_number = 1
        
        # Create filename
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        log_filename = f"{log_number:03d}_{timestamp}.log"
        log_filepath = logs_dir / log_filename
        
        # Create dedicated logger
        logger_name = f"drone_file_{self.drone_name}_{log_number}"
        file_logger = logging.getLogger(logger_name)
        file_logger.setLevel(logging.DEBUG)
        file_logger.handlers.clear()
        
        try:
            file_handler = logging.FileHandler(log_filepath, encoding='utf-8', mode='w')
            file_handler.setLevel(logging.DEBUG)
            
            file_formatter = logging.Formatter(
                fmt='%(asctime)s [%(levelname)s] [%(name)s] %(message)s',
                datefmt='%Y-%m-%d %H:%M:%S'
            )
            file_handler.setFormatter(file_formatter)
            
            file_logger.addHandler(file_handler)
            file_logger.propagate = False
            
            file_logger.info(f"Logger initialized for drone: {self.drone_name}")
            
            print(f"[{datetime.now().strftime('%H:%M:%S')}] [{Colors.GREEN}INFO{Colors.RESET}] [{self.drone_name}] Log file created: {log_filepath}")
            
        except Exception as e:
            print(f"[{datetime.now().strftime('%H:%M:%S')}] [{Colors.RED}ERROR{Colors.RESET}] [{self.drone_name}] Failed to create log file: {e}")
            null_handler = logging.NullHandler()
            file_logger.addHandler(null_handler)
        
        return file_logger
    
    def _format_console_message(self, level: str, message: str) -> str:
        """Format message for console output with colors."""
        timestamp = datetime.now().strftime('%H:%M:%S')
        color = self.level_colors.get(level, Colors.WHITE)
        
        return (f"[{timestamp}] "
                f"[{color}{level}{Colors.RESET}] "
                f"[{Colors.CYAN}{self.drone_name}{Colors.RESET}] "
                f"{message}")
    
    def _log_both(self, level: str, message: str, *args, **kwargs):
        """Log to both console (with colors) and file."""
        if args:
            try:
                formatted_message = message % args
            except (TypeError, ValueError):
                formatted_message = f"{message} {args}"
        else:
            formatted_message = message
        
        # Console output with colors
        console_message = self._format_console_message(level, formatted_message)
        print(console_message)
        
        # File output
        try:
            file_level = getattr(logging, level.upper())
            self.file_logger.log(file_level, formatted_message)
        except Exception:
            pass
    
    def debug(self, message: str, *args, **kwargs):
        """Log debug message."""
        self._log_both("DEBUG", message, *args, **kwargs)
    
    def info(self, message: str, *args, **kwargs):
        """Log info message."""
        self._log_both("INFO", message, *args, **kwargs)
    
    def warning(self, message: str, *args, **kwargs):
        """Log warning message."""
        self._log_both("WARNING", message, *args, **kwargs)
    
    def warn(self, message: str, *args, **kwargs):
        """Alias for warning."""
        self.warning(message, *args, **kwargs)
    
    def error(self, message: str, *args, **kwargs):
        """Log error message."""
        self._log_both("ERROR", message, *args, **kwargs)
    
    def fatal(self, message: str, *args, **kwargs):
        """Log fatal message."""
        self._log_both("FATAL", message, *args, **kwargs)
    
    def critical(self, message: str, *args, **kwargs):
        """Log critical message."""
        self._log_both("CRITICAL", message, *args, **kwargs)