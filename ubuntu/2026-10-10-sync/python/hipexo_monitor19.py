"""Compatibility entry point: both launch names use the integrated interface."""
import sys
import hipexo_monitor as _monitor
if __name__ == "__main__":
    _monitor.main()
else:
    sys.modules[__name__] = _monitor
