import os
import re
from typing import List, Dict, Optional
from pathlib import Path
import markdown
from bs4 import BeautifulSoup


class DocumentParser:
    """
    Parses markdown files and extracts content with structural information
    """

    def __init__(self):
        pass

    def parse_markdown_file(self, file_path: str) -> Dict:
        """
        Parse a markdown file and extract content with structural information
        """
        with open(file_path, 'r', encoding='utf-8') as file:
            content = file.read()

        # Extract metadata from frontmatter if present
        metadata = self._extract_frontmatter(content)

        # Extract structural information (headings)
        structure = self._extract_structure(content)

        # Get clean content without frontmatter
        clean_content = self._remove_frontmatter(content)

        # Extract title from the first heading or filename
        title = metadata.get('title') or self._extract_title(clean_content) or Path(file_path).stem

        # Determine chapter/section from file path
        path_parts = Path(file_path).parts
        chapter = self._extract_chapter_info(path_parts)
        section = self._extract_section_info(path_parts)

        return {
            'title': title,
            'content': clean_content,
            'chapter': chapter,
            'section': section,
            'file_path': file_path,
            'metadata': metadata,
            'structure': structure
        }

    def _extract_frontmatter(self, content: str) -> Dict:
        """
        Extract YAML frontmatter from markdown content
        """
        import yaml

        # Look for YAML frontmatter between --- delimiters
        frontmatter_match = re.match(r'^---\s*\n(.*?)\n---\s*\n', content, re.DOTALL)

        if frontmatter_match:
            try:
                frontmatter = yaml.safe_load(frontmatter_match.group(1))
                return frontmatter or {}
            except yaml.YAMLError:
                return {}

        return {}

    def _remove_frontmatter(self, content: str) -> str:
        """
        Remove YAML frontmatter from content
        """
        frontmatter_match = re.match(r'^---\s*\n(.*?)\n---\s*\n', content, re.DOTALL)

        if frontmatter_match:
            return content[frontmatter_match.end():]

        return content

    def _extract_structure(self, content: str) -> List[Dict]:
        """
        Extract structural information (headings) from markdown content
        """
        # Convert markdown to HTML to easily extract headings
        html = markdown.markdown(content)
        soup = BeautifulSoup(html, 'html.parser')

        structure = []
        for i, heading in enumerate(soup.find_all(['h1', 'h2', 'h3', 'h4', 'h5', 'h6'])):
            structure.append({
                'level': int(heading.name[1]),
                'text': heading.get_text().strip(),
                'position': i
            })

        return structure

    def _extract_title(self, content: str) -> Optional[str]:
        """
        Extract title from the first heading in the content
        """
        lines = content.split('\n')
        for line in lines:
            # Check for markdown heading pattern
            heading_match = re.match(r'^#+\s+(.+)', line)
            if heading_match:
                return heading_match.group(1).strip()

        return None

    def _extract_chapter_info(self, path_parts: tuple) -> str:
        """
        Extract chapter information from file path
        """
        # Look for common chapter-related directory names
        for part in path_parts:
            if 'chapter' in part.lower() or 'module' in part.lower():
                return part

        # If no chapter directory found, use the directory name
        if len(path_parts) > 1:
            return path_parts[-2]  # Parent directory of the file

        return 'unknown'

    def _extract_section_info(self, path_parts: tuple) -> str:
        """
        Extract section information from file path
        """
        file_name = path_parts[-1]
        # Remove file extension
        section = Path(file_name).stem
        return section


def scan_markdown_files(directory: str) -> List[str]:
    """
    Scan a directory for markdown files
    """
    markdown_files = []
    for root, dirs, files in os.walk(directory):
        for file in files:
            if file.lower().endswith(('.md', '.markdown')):
                markdown_files.append(os.path.join(root, file))

    return markdown_files