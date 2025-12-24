<!--
Sync Impact Report:
Version change: undefined → 1.0.0
Added sections: All principles and sections for AI-Driven Book + Embedded RAG Chatbot project
Removed sections: None
Templates requiring updates:
- .specify/templates/plan-template.md ✅ updated
- .specify/templates/spec-template.md ✅ updated
- .specify/templates/tasks-template.md ✅ updated
- .specify/templates/commands/*.md ✅ updated
- README.md ✅ updated
Follow-up TODOs: None
-->
# AI-Driven Book + Embedded RAG Chatbot Constitution

## Core Principles

### Technical Accuracy First
All content and implementations must be technically accurate from primary sources; All code examples must be fully tested and reproducible; Strict adherence to best practices and industry standards

### Developer-Centric Clarity
All documentation and code must be clear and accessible to software/AI developer audience; Complex concepts explained with practical examples; Comprehensive API documentation and usage guides

### Full Reproducibility (NON-NEGOTIABLE)
End-to-end setup and deployment must be fully reproducible from documentation; Installation instructions complete and tested on clean environments; All dependencies versioned and pinned where necessary

### Professional Publishing Standards
Book content meets professional publishing quality standards; Structured chapters with consistent formatting, glossary, and references; Proper grammar, spelling, and technical accuracy maintained

### Embedded RAG Excellence
RAG chatbot answers strictly from book content and selected text; No hallucinated or external answers allowed; Accurate retrieval and response generation with confidence scoring

### Modern Tech Stack Integration
Utilize modern technology stack: Docusaurus for book, OpenAI Agents/ChatKit for chatbot, Neon Postgres and Qdrant Cloud for data; All components must integrate seamlessly and securely

## Architecture and Infrastructure Requirements

Book built using Docusaurus + Spec-Kit Plus, deployed to GitHub Pages; Chatbot uses OpenAI Agents/ChatKit SDKs + FastAPI; Data storage with Neon Serverless Postgres + Qdrant Cloud; Minimum 6 chapters plus intro and conclusion required; Must include comprehensive installation instructions and architecture overview

## Development and Quality Standards

Written and developed entirely inside Claude Code; All changes must be tested end-to-end before merging; Code reviews required for all substantial changes; Documentation must be updated alongside code changes

## Governance

All implementations must strictly follow constitution principles; Changes to core architecture require explicit approval; All code must pass automated testing before merging; Book content must be factually accurate and properly sourced

**Version**: 1.0.0 | **Ratified**: 2025-12-22 | **Last Amended**: 2025-12-22