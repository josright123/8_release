#
# 這是一個用來分別編譯 extts.c, perout.c, stop.c 的 Makefile
#

# 編譯器設定
CC = gcc
# 編譯器旗標：
# -Wall: 顯示所有警告訊息
# -Wextra: 顯示額外的警告訊息
# -g: 產生除錯資訊，方便使用 gdb
CFLAGS = -Wall -Wextra -g

# 要產生的執行檔名稱
TARGETS = extts perout stop testptp phase one_shot

# .PHONY 用於宣告「偽目標」，這些目標不是真正的檔案名稱
# all 和 clean 是慣例用法
.PHONY: all clean

# 預設目標：當你只輸入 `make` 時，會執行這個目標
# 它的依賴是所有要產生的執行檔
all: $(TARGETS)

# --- 編譯規則 ---

# $@: 代表目標檔案 (例如 extts)
# $<: 代表第一個依賴檔案 (例如 extts.c)
# 這是一個通用的規則，make 會自動為 TARGETS 中的每一個目標尋找對應的 .c 檔
# 例如，當要建立 extts 時，make 會使用 extts.c
extts: extts.c
	$(CC) $(CFLAGS) -o $@ $<

perout: perout.c
	$(CC) $(CFLAGS) -o $@ $<

one_shot: one_shot.c
	$(CC) $(CFLAGS) -o $@ $<

phase: phase.c
	$(CC) $(CFLAGS) -o $@ $<

stop: stop.c
	$(CC) $(CFLAGS) -o $@ $<

testptp: testptp.c
	$(CC) $(CFLAGS) -o $@ $< -lrt

# --- 清理規則 ---

# 這個目標用來刪除所有編譯產生的執行檔
clean:
	@echo "正在清理編譯產生的檔案..."
	rm -f $(TARGETS)
	@echo "清理完成！"
