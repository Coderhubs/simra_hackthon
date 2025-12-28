from pydantic import BaseModel, Field
from typing import List, Optional, Dict, Any
from enum import Enum


class ContentChunk(BaseModel):
    id: str
    text: str
    metadata: Dict[str, Any]


class LiveContentRequest(BaseModel):
    content: str
    metadata: Optional[Dict[str, Any]] = Field(default_factory=dict)


class IngestionResponse(BaseModel):
    success: bool
    message: str
    ids: Optional[List[str]] = None


class QueryWithMetadata(BaseModel):
    query: str
    filters: Optional[Dict[str, Any]] = Field(default_factory=dict)
    selected_text: Optional[str] = None
    top_k: int = 5


class IngestRequest(BaseModel):
    content: str
    metadata: Optional[Dict[str, Any]] = Field(default_factory=dict)


class ChatRequest(BaseModel):
    message: str
    history: Optional[List[Dict[str, str]]] = Field(default_factory=list)
    filters: Optional[Dict[str, Any]] = Field(default_factory=dict)
    selected_text: Optional[str] = None


class ChatResponse(BaseModel):
    response: str
    context: Optional[List[Dict[str, Any]]] = None


class HTTPValidationError(BaseModel):
    detail: Optional[List[Any]] = None


class ValidationError(BaseModel):
    loc: List[str]
    msg: str
    type: str