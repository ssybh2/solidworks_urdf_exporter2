"""Autodesk Inventor (.iam/.ipt) backend entry point for sw2robot."""
from .inventor_backend.extract import extract_inventor, main
from .inventor_backend.com import Inventor, InventorUnavailable

__all__ = ["Inventor", "InventorUnavailable", "extract_inventor", "main"]

if __name__ == "__main__":
    main()
