import os
from typing import List, Dict, Optional
from pathlib import Path
from .document_parser import DocumentParser, scan_markdown_files


class FileScanner:
    """
    Scans and processes markdown files from the Docusaurus documentation directory
    """

    def __init__(self, base_path: str = "docusaurus/docs"):
        self.base_path = base_path
        self.parser = DocumentParser()

    def scan_and_parse_documents(self) -> List[Dict]:
        """
        Scan the documentation directory and parse all markdown files
        """
        if not os.path.exists(self.base_path):
            raise FileNotFoundError(f"Documentation directory not found: {self.base_path}")

        markdown_files = scan_markdown_files(self.base_path)
        documents = []

        for file_path in markdown_files:
            try:
                document = self.parser.parse_markdown_file(file_path)
                documents.append(document)
            except Exception as e:
                print(f"Error parsing file {file_path}: {str(e)}")
                continue

        return documents

    def validate_document(self, document: Dict) -> bool:
        """
        Validate document structure and content
        """
        required_fields = ['title', 'content', 'chapter', 'section', 'file_path']
        for field in required_fields:
            if field not in document or not document[field]:
                return False

        # Check content length
        if len(document['content'].strip()) < 10:
            return False

        return True

    def get_document_stats(self, documents: List[Dict]) -> Dict:
        """
        Get statistics about the parsed documents
        """
        total_docs = len(documents)
        valid_docs = sum(1 for doc in documents if self.validate_document(doc))
        total_chars = sum(len(doc['content']) for doc in documents)
        unique_chapters = len(set(doc['chapter'] for doc in documents))

        return {
            'total_documents': total_docs,
            'valid_documents': valid_docs,
            'invalid_documents': total_docs - valid_docs,
            'total_characters': total_chars,
            'unique_chapters': unique_chapters,
            'average_length': total_chars // total_docs if total_docs > 0 else 0
        }


def main():
    """
    Main function to demonstrate file scanning
    """
    # Use the docusaurus docs directory by default, or allow override
    scanner = FileScanner()
    documents = scanner.scan_and_parse_documents()

    print(f"Found {len(documents)} documents")
    stats = scanner.get_document_stats(documents)
    print(f"Statistics: {stats}")

    # Print first document as example
    if documents:
        print(f"\nFirst document example:")
        print(f"Title: {documents[0]['title']}")
        print(f"Chapter: {documents[0]['chapter']}")
        print(f"Section: {documents[0]['section']}")
        print(f"Content preview: {documents[0]['content'][:200]}...")


if __name__ == "__main__":
    main()