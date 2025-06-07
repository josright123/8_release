# Commit Message Style Guide

This document provides a standard format for writing clear and consistent commit messages across the DM9051A PTP driver project.

## ✅ Commit Message Format

### 1. Header (Title Line)
- Starts with a lowercase action verb
- Limited to 50–72 characters
- Describes the change concisely
- No period at the end

**Example:**

### 2. Body (Detailed Description)
Provides a more detailed explanation including:
- What was changed
- Why it was done
- Breakdown of the components

**Example:**

### 3. Common Commit Types

| Type      | Description                                 |
|-----------|---------------------------------------------|
| `feat`    | New feature (e.g., PTP support)            |
| `fix`     | Bug fix (e.g., SPI timing issue)           |
| `docs`    | Documentation only changes                  |
| `style`   | Code formatting (follows .clang-format)    |
| `refactor`| Code restructuring                          |
| `chore`   | Build scripts, Makefile updates            |
| `test`    | Adding or updating tests                    |
| `perf`    | Performance improvements                    |

## Project-Specific Guidelines

### For DM9051A Driver:
- Reference specific modules: `ptp`, `spi`, `netdev`, `ethtool`
- Mention hardware features: `timestamp`, `one-step`, `two-step`
- Include register names when relevant: `TCR`, `RSR`, `PTPTSLR`