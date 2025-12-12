#!/usr/bin/env python3
"""
YOLO 모델/데이터셋 동기화 도구 (OpenCV GUI)

사용법:
    python scripts/sync_yolo_model.py

키 조작:
    U: 데이터셋 업로드 (로컬 → 서버)
    D: 모델 다운로드 (서버 → 로컬)
    Q/ESC: 종료
"""

import os
import sys
import cv2
import numpy as np
import threading
from pathlib import Path

# 프로젝트 루트 디렉토리
project_root = Path(__file__).parent.parent.parent
sys.path.insert(0, str(project_root))

from dotenv import load_dotenv
load_dotenv(project_root / ".env")

import paramiko


class SyncTool:
    def __init__(self):
        # 설정
        self.host = os.getenv("REMOTE_MODEL_HOST", "")
        self.user = os.getenv("REMOTE_MODEL_USER", "")
        self.password = os.getenv("REMOTE_MODEL_PASSWORD", "")
        self.model_name = os.getenv("YOLO_DASHBOARD_MODEL", "my_yolo.pt")
        self.remote_model = os.getenv("REMOTE_MODEL_PATH", "~/web_server/models/my_yolo.pt")
        
        self.local_dataset = project_root / "auto_labeling" / "dataset"
        self.local_model = project_root / "models" / self.model_name
        self.remote_dataset = "~/web_server/auto_labeling/dataset"
        
        # 상태
        self.status = "Ready"
        self.progress = 0
        self.is_running = False
        self.running = True
        
        # 버튼 영역 정의 (x1, y1, x2, y2)
        self.upload_btn_area = (50, 120, 230, 180)
        self.download_btn_area = (270, 120, 450, 180)
        
        # 파일 개수
        imgs = list((self.local_dataset / "images").glob("*.jpg")) if (self.local_dataset / "images").exists() else []
        lbls = list((self.local_dataset / "labels").glob("*.txt")) if (self.local_dataset / "labels").exists() else []
        self.file_count = f"Local: {len(imgs)} images, {len(lbls)} labels"
    
    def mouse_callback(self, event, x, y, flags, param):
        """마우스 클릭 이벤트 처리"""
        if event == cv2.EVENT_LBUTTONDOWN:
            # Upload 버튼 클릭 확인
            ux1, uy1, ux2, uy2 = self.upload_btn_area
            if ux1 <= x <= ux2 and uy1 <= y <= uy2:
                if not self.is_running:
                    threading.Thread(target=self.upload_dataset, daemon=True).start()
            
            # Download 버튼 클릭 확인
            dx1, dy1, dx2, dy2 = self.download_btn_area
            if dx1 <= x <= dx2 and dy1 <= y <= dy2:
                if not self.is_running:
                    threading.Thread(target=self.download_model, daemon=True).start()
    
    def draw_ui(self):
        """UI 그리기"""
        img = np.zeros((400, 500, 3), dtype=np.uint8)
        img[:] = (45, 45, 45)  # 다크 배경
        
        # 제목
        cv2.putText(img, "YOLO Sync Tool", (150, 50), 
                   cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        
        # 서버 정보
        cv2.putText(img, f"Server: {self.user}@{self.host}", (20, 90),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (150, 150, 150), 1)
        
        # 버튼들
        # Upload 버튼 (U)
        cv2.rectangle(img, (50, 120), (230, 180), (240, 156, 33), -1)  # 파란색
        cv2.putText(img, "[U] Upload Dataset", (60, 158),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        
        # Download 버튼 (D)
        cv2.rectangle(img, (270, 120), (450, 180), (76, 175, 80), -1)  # 초록색
        cv2.putText(img, "[D] Download Model", (275, 158),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2)
        
        # 프로그래스 바
        bar_x, bar_y, bar_w, bar_h = 50, 220, 400, 30
        cv2.rectangle(img, (bar_x, bar_y), (bar_x + bar_w, bar_y + bar_h), (60, 60, 60), -1)
        fill_w = int(bar_w * self.progress / 100)
        if fill_w > 0:
            color = (76, 175, 80) if "Complete" in self.status else (240, 156, 33)
            cv2.rectangle(img, (bar_x, bar_y), (bar_x + fill_w, bar_y + bar_h), color, -1)
        cv2.rectangle(img, (bar_x, bar_y), (bar_x + bar_w, bar_y + bar_h), (100, 100, 100), 2)
        
        # 프로그래스 텍스트
        cv2.putText(img, f"{int(self.progress)}%", (bar_x + bar_w//2 - 20, bar_y + 22),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        
        # 상태
        status_color = (80, 200, 120) if "Complete" in self.status else \
                      (80, 150, 240) if "ing" in self.status else \
                      (80, 80, 240) if "Error" in self.status else (150, 150, 150)
        cv2.putText(img, self.status, (50, 290),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, status_color, 1)
        
        # 파일 개수
        cv2.putText(img, self.file_count, (50, 330),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (100, 100, 100), 1)
        
        # 도움말
        cv2.putText(img, "Press Q or ESC to quit", (150, 380),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (100, 100, 100), 1)
        
        return img
    
    def upload_dataset(self):
        """데이터셋 업로드"""
        if self.is_running:
            return
        self.is_running = True
        self.progress = 0
        
        try:
            self.status = "Connecting..."
            
            ssh = paramiko.SSHClient()
            ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
            ssh.connect(self.host, username=self.user, password=self.password, timeout=10)
            
            # 홈 디렉토리 확장
            stdin, stdout, _ = ssh.exec_command("echo $HOME")
            home = stdout.read().decode().strip()
            remote = self.remote_dataset.replace("~", home)
            
            # 기존 데이터 삭제 후 새로 생성
            self.status = "Deleting old data..."
            ssh.exec_command(f"rm -rf {remote}/images {remote}/labels")
            stdin, stdout, stderr = ssh.exec_command(f"mkdir -p {remote}/images {remote}/labels")
            stdout.read()  # 완료 대기
            
            sftp = ssh.open_sftp()
            
            imgs = list((self.local_dataset / "images").glob("*.jpg"))
            lbls = list((self.local_dataset / "labels").glob("*.txt"))
            total = len(imgs) + len(lbls)
            done = 0
            
            self.status = f"Uploading images... (0/{len(imgs)})"
            for f in imgs:
                sftp.put(str(f), f"{remote}/images/{f.name}")
                done += 1
                self.progress = done / total * 100
                if done % 50 == 0:
                    self.status = f"Uploading images... ({done}/{len(imgs)})"
            
            self.status = f"Uploading labels... (0/{len(lbls)})"
            lbl_start = done
            for f in lbls:
                sftp.put(str(f), f"{remote}/labels/{f.name}")
                done += 1
                self.progress = done / total * 100
                if (done - lbl_start) % 50 == 0:
                    self.status = f"Uploading labels... ({done - lbl_start}/{len(lbls)})"
            
            # classes.yaml
            cls_file = project_root / "auto_labeling" / "classes.yaml"
            if cls_file.exists():
                sftp.put(str(cls_file), f"{remote}/../classes.yaml")
            
            sftp.close()
            ssh.close()
            
            self.progress = 100
            self.status = f"Upload Complete! ({len(imgs)} imgs, {len(lbls)} lbls)"
            
        except Exception as e:
            self.status = f"Error: {str(e)[:50]}"
        finally:
            self.is_running = False
    
    def download_model(self):
        """모델 다운로드"""
        if self.is_running:
            return
        self.is_running = True
        self.progress = 0
        
        try:
            self.status = "Connecting..."
            
            ssh = paramiko.SSHClient()
            ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
            ssh.connect(self.host, username=self.user, password=self.password, timeout=10)
            
            remote = self.remote_model
            if remote.startswith("~"):
                stdin, stdout, _ = ssh.exec_command("echo $HOME")
                home = stdout.read().decode().strip()
                remote = remote.replace("~", home)
            
            sftp = ssh.open_sftp()
            
            try:
                stat = sftp.stat(remote)
                size = stat.st_size
            except:
                raise Exception(f"Remote file not found: {self.remote_model}")
            
            self.status = "Downloading model..."
            self.local_model.parent.mkdir(parents=True, exist_ok=True)
            
            def callback(transferred, total):
                self.progress = transferred / total * 100
            
            sftp.get(remote, str(self.local_model), callback=callback)
            
            sftp.close()
            ssh.close()
            
            self.progress = 100
            self.status = f"Download Complete! ({size/1024/1024:.1f} MB)"
            
        except Exception as e:
            self.status = f"Error: {str(e)[:50]}"
        finally:
            self.is_running = False
    
    def run(self):
        """메인 루프"""
        cv2.namedWindow("YOLO Sync Tool", cv2.WINDOW_AUTOSIZE)
        cv2.setMouseCallback("YOLO Sync Tool", self.mouse_callback)
        
        while self.running:
            img = self.draw_ui()
            cv2.imshow("YOLO Sync Tool", img)
            
            key = cv2.waitKey(50) & 0xFF
            
            if key == ord('q') or key == 27:  # Q or ESC
                self.running = False
            elif key == ord('u') or key == ord('U'):
                threading.Thread(target=self.upload_dataset, daemon=True).start()
            elif key == ord('d') or key == ord('D'):
                threading.Thread(target=self.download_model, daemon=True).start()
        
        cv2.destroyAllWindows()


if __name__ == "__main__":
    print("=" * 50)
    print("🤖 YOLO Sync Tool (OpenCV)")
    print("=" * 50)
    print("U: Upload Dataset")
    print("D: Download Model")
    print("Q/ESC: Quit")
    print("=" * 50)
    
    tool = SyncTool()
    tool.run()
