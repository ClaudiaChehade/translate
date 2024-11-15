import os
import uuid
from weasyprint import HTML, CSS
from weasyprint.logger import LOGGER

def find_html_files(directory):
    html_files = []
    for root, dirs, files in os.walk(directory):
        for file in files:
            if file.endswith('.html') or file.endswith('.htm'):
                html_files.append(os.path.join(root, file))
    return sorted(html_files)

def combine_html_files(html_files):
    combined_html = ""
    for file in html_files:
        with open(file, 'r', encoding='utf-8') as f:
            combined_html += f.read() + "\n"
    return combined_html

def convert_html_to_pdf(html_content, output_pdf):
    pdf_identifier = str(uuid.uuid4())
    
    # Create the HTML object
    html = HTML(string=html_content)
    
    # PDF options
    pdf_options = {
        'version': '1.7',  # PDF version
        'identifier': pdf_identifier  # Unique identifier
    }
    
    LOGGER.info('Generating PDF with options: %s', pdf_options)
    html.write_pdf(output_pdf, zoom=1.0, presentational_hints=True, **pdf_options)

# Set the directory containing your HTML files
directory = '/home/lac2pl/workspaces/xflow-libs-overwriting-tdp/scripts/release/translate_from_html/PER-v1-20240716_1612'

# Find all HTML files
html_files = find_html_files(directory)

# Combine HTML files into one string
combined_html_content = combine_html_files(html_files)

# Set the output PDF file name
output_pdf = '/home/lac2pl/workspaces/xflow-libs-overwriting-tdp/scripts/release/translate_from_html/wave3.pdf'

# Convert to PDF
convert_html_to_pdf(combined_html_content, output_pdf)

print(f'PDF generated successfully: {output_pdf}')
