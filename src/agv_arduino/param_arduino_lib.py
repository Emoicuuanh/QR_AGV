import git         # Thư viện GitPython để lấy thông tin commit của các thư viện
import configparser  # Thư viện để đọc file cấu hình platformio.ini

# Hàm lấy thông tin commit của thư viện
def get_current_commit_data():
    LIBS_info = ""  # Chuỗi để lưu thông tin commit của các thư viện

    # Đọc file cấu hình platformio.ini
    config = configparser.ConfigParser()
    config.read("platformio.ini")

    # Lấy danh sách thư mục chứa các thư viện ngoài (nếu có)
    lib_extra_dirs = config["env:megaatmega2560"]["lib_extra_dirs"]

    for line in lib_extra_dirs.split("\n"):  # Duyệt từng dòng trong danh sách thư viện
        i = 0  # Biến đếm số thứ tự thư viện
        if line.strip():  # Nếu dòng không rỗng
            i += 1  # Tăng biến đếm
            repo = git.Repo(f"{line.strip()}")  # Mở repository Git trong thư viện
            head_commit = repo.head.commit  # Lấy commit mới nhất
            commit_hash = head_commit.hexsha  # Lấy hash của commit
            commit_message = head_commit.message.strip()  # Lấy nội dung commit
            commit_author = head_commit.author.name  # Lấy tên tác giả commit
            commit_date = head_commit.authored_datetime.strftime("%Y-%m-%d %H:%M:%S")  # Lấy ngày commit

            # Ghép thông tin thành một chuỗi
            LIBS_info += (
                " No."
                + str(i)
                + " - "
                + "commit_hash: "
                + str(commit_hash[:8])  # Chỉ lấy 8 ký tự đầu của hash
                + " - "
                + "commit_message: "
                + str(commit_message)
                + " - "
                + "commit_author: "
                + str(commit_author)
                + " - "
                + "commit_date: "
                + str(commit_date)
            )
    return LIBS_info  # Trả về chuỗi chứa thông tin các commit của thư viện


# Hàm lấy danh sách thư viện từ lib_deps
def get_LIBS():
    lib_extra_dirs_list = ""  # Chuỗi lưu danh sách thư viện
    config = configparser.ConfigParser()
    config.read("platformio.ini")  # Đọc file cấu hình

    # Lấy danh sách thư viện trong mục lib_deps
    lib_extra_dirs = config["env:megaatmega2560"]["lib_deps"]

    for line in lib_extra_dirs.split("\n"):  # Duyệt từng dòng
        if line.strip():  # Nếu dòng không rỗng
            lib_extra_dirs_list += str(line.strip()) + " - "  # Thêm vào danh sách

    return lib_extra_dirs_list  # Trả về danh sách thư viện


# In ra macro chứa danh sách thư viện
print("'-D LIBS=\"%s\"'" % get_LIBS())

# Gọi hàm lấy thông tin commit của thư viện
commit_info = get_current_commit_data()

# In ra macro chứa thông tin commit của các thư viện
print("'-D ARDUINO_COMMON=\"%s\"'" % commit_info)
