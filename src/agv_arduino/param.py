import git  # Thư viện để thao tác với Git

def get_current_commit_data():
    # Mở repository Git ở thư mục hiện tại
    repo = git.Repo(".")
    
    # Lấy commit mới nhất (HEAD commit)
    head_commit = repo.head.commit
    
    # Lấy thông tin của commit
    commit_hash = head_commit.hexsha  # Hash của commit
    commit_message = head_commit.message.strip()  # Thông điệp của commit
    commit_author = head_commit.author.name  # Tên tác giả của commit
    commit_date = head_commit.authored_datetime.strftime("%Y-%m-%d %H:%M:%S")  # Ngày giờ của commit
    branch_name = repo.active_branch.name  # Tên nhánh hiện tại
    
    # Trả về các thông tin cần thiết
    return commit_hash, commit_message, commit_author, commit_date, branch_name

# Lấy thông tin commit
commit_hash, commit_message, commit_author, commit_date, branch_name = get_current_commit_data()

# In ra thông tin dưới dạng macro
print("'-D COMMIT_HASH=\"%s\"'" % commit_hash[:8])  # Chỉ in 8 ký tự đầu của commit hash
print("'-D COMMIT_MESSAGE=\"%s\"'" % commit_message)
print("'-D COMMIT_AUTHOR=\"%s\"'" % commit_author)
print("'-D COMMIT_DATE=\"%s\"'" % commit_date)
print("'-D BRANCH_NAME=\"%s\"'" % branch_name)