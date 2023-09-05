from __future__ import annotations
from typing import Any, Tuple
from queue import Queue
from .tkinter_objects import TkLabel

class InterfaceQueue(Queue):

    def __init__(self, maxsize: int = 0) -> None:
        
        super().__init__(maxsize)

    def put(self, label: TkLabel, data: Any, block: bool = True, timeout: float | None = None) -> None:
        
        return super().put((label, data), block, timeout)
    
    def get(self, block: bool = True, timeout: float | None = None) -> Tuple[TkLabel, Any]:
        
        return super().get(block, timeout)