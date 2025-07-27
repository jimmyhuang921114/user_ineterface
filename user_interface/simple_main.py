from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from fastapi.staticfiles import StaticFiles
from contextlib import asynccontextmanager
from pathlib import Path

# API 路由匯入
from route.routes_medicine import router as medicine_routes

# 初始化資料庫
from database.medicine_db import init_db as init_medicine_db

@asynccontextmanager
async def lifespan(app: FastAPI):
    init_medicine_db()
    yield

app = FastAPI(title="Hospital System API", lifespan=lifespan)

# 添加CORS中間件
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# 設定資料夾路徑
base_dir = Path(__file__).resolve().parent
static_dir = base_dir / "static"

# 掛載靜態資源
app.mount("/css", StaticFiles(directory=static_dir / "css"), name="css")
app.mount("/js", StaticFiles(directory=static_dir / "js"), name="js")
app.mount("/html", StaticFiles(directory=static_dir / "html"), name="html")

# 掛載 API 路由
app.include_router(medicine_routes, prefix="/api", tags=["Medicine API"])

# 簡單的根路由
@app.get("/")
async def read_root():
    return {"message": "醫院藥物管理系統API", "status": "running"}

@app.get("/Medicine.html")
async def get_medicine_page():
    return StaticFiles(directory=static_dir / "html").get_response("Medicine.html")