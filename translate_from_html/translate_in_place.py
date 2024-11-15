import os
import re
import html
import argparse
from googletrans import Translator

def to_html_escape(text):
    return ''.join(f'&#{ord(char)};' for char in text)

def translate_html(file_path):
    """
    Translates Chinese text within an HTML file to English and saves the changes in place.
    """
    translator = Translator()

    with open(file_path, 'r', encoding='utf-8') as f:
        html_content = f.read()

    # Decode HTML entities
    decoded_content = html.unescape(html_content)

    # Find all text within HTML elements that contain Chinese characters using regex
    chinese_texts = re.findall(r'>([^<]*[\u4e00-\u9fff]+[^<]*)<', decoded_content)

    # Translate Chinese text to English and replace it in the HTML content
    for text in set(chinese_texts):
        try :
            if text.startswith("//"):
                text = text.replace("//", "")
                translated_text = translator.translate(text, src='zh-cn', dest='en').text
                translated_text = "//" + translated_text
                print(f"  Translated text: {text} -> {translated_text}")
                decoded_content = decoded_content.replace(text, f" {translated_text} ")
            else:
                translated_text = translator.translate(text, src='zh-cn', dest='en').text
                print(f"  Translated text: {text} -> {translated_text}")
                decoded_content = decoded_content.replace(text, f" {translated_text} ")
        except Exception:
            print(f"Can not translate {text}")

    encoded_content = html.escape(decoded_content)

    # Save the modified HTML to the same file
    with open(file_path, 'w', encoding='utf-8') as f:
        f.write(decoded_content)

    print(f'Translated HTML saved as {file_path}')

def translate_folder(folder_path):
    """
    Recursively iterates over all files in the given folder and its subfolders,
    translating any HTML files found.
    """
    file_counter = 0
    for root, _, files in os.walk(folder_path):
        for file in files:
            if file.endswith('.html'):
                file_counter = file_counter + 1
    print(f"Will translate this number of files: {file_counter}")

    file_counter = 0
    for root, _, files in os.walk(folder_path):
        for file in files:
            if file.endswith('.html'):
                file_path = os.path.join(root, file)
                print(f"{file_counter}. Translating {file_path}")
                translate_html(file_path)
                file_counter = file_counter + 1

def main():
    parser = argparse.ArgumentParser(description='Translate all HTML files in a given folder.')
    parser.add_argument('folder', type=str, help='The path to the folder containing HTML files to be translated.')
    args = parser.parse_args()
    
    folder_path = args.folder
    folder_path = "/home/lac2pl/workspaces/translate/translate_from_html/PER-v1-20240716_1612_to_be_translated"
    
    if os.path.isdir(folder_path):
        translate_folder(folder_path)
    else:
        print(f"The path {folder_path} is not a valid directory.")

if __name__ == '__main__':
    main()
