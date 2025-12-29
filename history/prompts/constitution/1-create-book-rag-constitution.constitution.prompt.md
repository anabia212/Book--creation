---
id: 1
title: Create Book RAG Constitution
stage: constitution
date: 2025-12-29
surface: agent
model: Claude Sonnet 4.5
feature: none
branch: main
user: user
command: /sp.constitution
labels:
  - constitution
  - spec-driven
  - book
  - rag
links:
  SPEC: null
  TICKET: null
  ADR: null
  PR: null
files:
  - .specify/memory/constitution.md
tests: []
---

# Prompt

```
/sp.constitution

Project:
Spec-Driven Technical Book with Integrated RAG Chatbot

Core Principles

Spec-first development using Spec-Kit Plus

Accuracy and strict grounding in source content

Clarity for AI/software engineering audience

Reproducible builds and deployments

Zero hallucination tolerance

Deliverables
1. AI / Spec-Driven Book

Written with Claude Code

Built using Docusaurus

Deployed to GitHub Pages

Technical, structured, progressive content

All claims verifiable and traceable

2. Integrated RAG Chatbot

Embedded within the published book

Tech stack:

OpenAI Agents / ChatKit SDKs

FastAPI backend

Neon Serverless Postgres

Qdrant Cloud (free tier)

Capabilities:

Answer questions about full book

Answer using only user-selected text

Responses strictly grounded in indexed content

Standards & Constraints

Spec required before implementation

Modular, maintainable architecture

Cloud free-tier compliant

Secure API design

Deterministic ingestion and embedding pipeline

Documentation

Spec-Kit Plus specs for:

Book structure

Build & deployment

RAG architecture

API contracts

Clear setup and deployment instructions

Success Criteria

Book live on GitHub Pages

RAG chatbot fully functional and embedded

Accurate, grounded, context-restricted answers

Fully reproducible system

Passes accuracy and spec-compliance review
```

# Response

Created comprehensive constitution for the Spec-Driven Technical Book with Integrated RAG Chatbot project. The constitution includes:

## Core Principles
1. Spec-First Development - All development begins with comprehensive specifications
2. Accuracy and Strict Grounding - Zero hallucination tolerance for content and responses
3. Clarity for AI/Software Engineering Audience - Technical precision and accessibility
4. Reproducible Systems - Deterministic and reproducible processes
5. Modular Architecture - Clean separation of concerns between components
6. Free-Tier Compliance - Cost-effective architecture decisions

## Additional Sections
- Standards & Constraints with specific tech stack requirements
- Development Workflow with mandatory steps
- Governance with amendment procedures

The constitution has been written to `.specify/memory/constitution.md` with version 1.0.0, ratified on 2025-01-01 and last amended on 2025-12-29.

# Outcome

The project constitution has been successfully created and will guide all future development decisions for the Spec-Driven Technical Book with Integrated RAG Chatbot project, ensuring alignment with the core principles of spec-first development, accuracy, clarity, reproducibility, modularity, and cost-effectiveness.

# Evaluation

The constitution successfully captures all the core principles specified in the user requirements and provides a governance framework that will ensure consistent decision-making throughout the project lifecycle.