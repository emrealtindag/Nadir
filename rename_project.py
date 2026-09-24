import os
import glob

def rename_project(root_dir):
    # 1. Rename contents in all text files
    extensions = ['.py', '.md', '.toml', '.ini', '.yaml', '.sh', '.cff', '.cpp', '.hpp', '.json']
    
    for dirpath, dirnames, filenames in os.walk(root_dir):
        # Skip git and cache directories
        if '.git' in dirpath or '__pycache__' in dirpath or '.pytest_cache' in dirpath:
            continue
            
        for filename in filenames:
            if any(filename.endswith(ext) for ext in extensions) or filename == 'LICENSE' or filename == 'NOTICE':
                filepath = os.path.join(dirpath, filename)
                
                # Exclude this script itself
                if filename == 'rename_project.py' or filename.endswith('.lock'):
                    continue
                    
                try:
                    with open(filepath, 'r', encoding='utf-8') as f:
                        content = f.read()
                        
                    if 'scandium' in content or 'Scandium' in content or 'SCANDIUM' in content:
                        new_content = content.replace('Scandium', 'Nadir')
                        new_content = new_content.replace('scandium', 'nadir')
                        new_content = new_content.replace('SCANDIUM', 'NADIR')
                        
                        with open(filepath, 'w', encoding='utf-8') as f:
                            f.write(new_content)
                        print(f"Updated content in: {filepath}")
                except Exception as e:
                    print(f"Could not process {filepath}: {e}")

    # 2. Rename directories
    src_dir = os.path.join(root_dir, 'src', 'scandium')
    dest_dir = os.path.join(root_dir, 'src', 'nadir')
    
    if os.path.exists(src_dir):
        os.rename(src_dir, dest_dir)
        print(f"Renamed directory {src_dir} to {dest_dir}")
        
    print("Project rename to Nadir complete!")

if __name__ == "__main__":
    rename_project(r"c:\Users\Emre\Downloads\scandium-main")
