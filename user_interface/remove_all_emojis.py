#!/usr/bin/env python3
"""
emoji
Remove All Emojis Script
"""

import os
import re
import glob
from pathlib import Path

# emoji
EMOJI_PATTERN = re.compile(
    "["
    "\U0001F600-\U0001F64F"  # emoticons
    "\U0001F300-\U0001F5FF"  # symbols & pictographs
    "\U0001F680-\U0001F6FF"  # transport & map symbols
    "\U0001F1E0-\U0001F1FF"  # flags (iOS)
    "\U00002702-\U000027B0"  # Dingbats
    "\U000024C2-\U0001F251"
    "\U0001F900-\U0001F9FF"  # Supplemental Symbols and Pictographs
    "\U0001FA70-\U0001FAFF"  # Symbols and Pictographs Extended-A
    "]+",
    flags=re.UNICODE
)

def remove_emojis_from_text(text):
    """emoji"""
    # emoji
    cleaned_text = EMOJI_PATTERN.sub('', text)

    #
    lines = cleaned_text.split('\n')
    cleaned_lines = []

    for line in lines:
        #
        if line.strip():
            #
            leading_spaces = len(line) - len(line.lstrip())
            content = line.strip()
            cleaned_lines.append(' ' * leading_spaces + content)
        else:
            cleaned_lines.append('')

    return '\n'.join(cleaned_lines)

def process_file(file_path):
    """"""
    try:
        #
        with open(file_path, 'r', encoding='utf-8') as f:
            original_content = f.read()

        # emoji
        cleaned_content = remove_emojis_from_text(original_content)

        #
        if original_content != cleaned_content:
            #
            with open(file_path, 'w', encoding='utf-8') as f:
                f.write(cleaned_content)

            # emoji
            emoji_count = len(EMOJI_PATTERN.findall(original_content))
            print(f": {file_path} ( {emoji_count} emoji)")
            return True
        else:
            print(f": {file_path} (emoji)")
            return False

    except Exception as e:
        print(f" {file_path}: {e}")
        return False

def get_all_text_files(directory):
    """"""
    extensions = [
        '*.py', '*.md', '*.html', '*.css', '*.js',
        '*.json', '*.txt', '*.sh', '*.yml', '*.yaml'
    ]

    all_files = []

    for ext in extensions:
        # glob
        pattern = os.path.join(directory, '**', ext)
        files = glob.glob(pattern, recursive=True)
        all_files.extend(files)

    #
    excluded_dirs = ['__pycache__', '.git', 'node_modules', '.venv']

    filtered_files = []
    for file_path in all_files:
        path_parts = Path(file_path).parts
        if not any(excluded_dir in path_parts for excluded_dir in excluded_dirs):
            filtered_files.append(file_path)

    return sorted(filtered_files)

def main():
    """"""
    print("emoji")
    print("=" * 50)

    #
    current_dir = os.getcwd()
    print(f": {current_dir}")

    #
    text_files = get_all_text_files(current_dir)

    if not text_files:
        print("")
        return

    print(f" {len(text_files)} ")
    print("-" * 50)

    #
    processed_count = 0
    modified_count = 0

    for file_path in text_files:
        processed_count += 1
        if process_file(file_path):
            modified_count += 1

    print("-" * 50)
    print(f"!")
    print(f": {processed_count}")
    print(f": {modified_count}")
    print(f": {processed_count - modified_count}")

if __name__ == "__main__":
    main()