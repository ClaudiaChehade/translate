import re
from googletrans import Translator
import html


def to_html_escape(text):
    return ''.join(f'&#{ord(char)};' for char in text)

# Initialize the translator
translator = Translator()

# Load the HTML content from a file
input_file = '/home/lac2pl/workspaces/xflow-libs-overwriting-tdp/scripts/release/translate_from_html/[PI2303] Parking Viper__Perception架构框图Refine-v1-20240716_1327 copy/2877927246_[PI2303]_Parking_Viper__Perception架构框图Refine.html' 
# input_file = '/home/lac2pl/workspaces/xflow-libs-overwriting-tdp/scripts/release/translate_from_html/[PI2303] Parking Viper__Perception架构框图Refine-v1-20240716_1327/2877927246_[PI2303]_Parking_Viper__Perception架构框图Refine.html' 
output_file = '/home/lac2pl/workspaces/xflow-libs-overwriting-tdp/scripts/release/translate_from_html/[PI2303] Parking Viper__Perception架构框图Refine-v1-20240716_1327/2877927246_[PI2303]_Parking_Viper__Perception架构框图Refine_copy.html'  


with open(input_file, 'r', encoding='utf-8') as f:
    html_content = f.read()

# Decode HTML entities
decoded_content = html.unescape(html_content)

# Find all Chinese text using regex
# chinese_texts = re.findall(r'[\u4e00-\u9fff]+', decoded_content)
chinese_texts = re.findall(r'(<.*?>.*?[\u4e00-\u9fff]+.*?</.*?>)', decoded_content, re.DOTALL)

# Translate Chinese text to English and replace it in the HTML content
for text in set(chinese_texts):
    translated_text = translator.translate(text, src='zh-cn', dest='en').text
    print(f"-------------------")
    print(f"translated_text: {translated_text}")
    original_chinese_text = to_html_escape(text)
    html_content = html_content.replace(original_chinese_text, f" {translated_text} ")

# Save the modified HTML to a new file
with open(output_file, 'w', encoding='utf-8') as f:
    f.write(html_content)

print(f'Translated HTML saved as {output_file}')