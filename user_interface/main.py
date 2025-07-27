from fastapi import FastAPI, Request
from fastapi.responses import HTMLResponse
from fastapi.staticfiles import StaticFiles
from fastapi.templating import Jinja2Templates
from fastapi.middleware.cors import CORSMiddleware
from contextlib import asynccontextmanager
from pathlib import Path

# API 路由匯入
from route.routes_medicine import router as medicine_routes
from route.routes_prescription import router as prescription_routes

# 初始化資料庫
from database.medicine_db import init_db as init_medicine_db
from database.prescription_db import init_db as init_prescription_db

@asynccontextmanager
async def lifespan(app: FastAPI):
    init_medicine_db()
    init_prescription_db()
    yield

app = FastAPI(title="Hospital System API", lifespan=lifespan)

# 添加CORS中間件
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # 在生產環境中應該設置具體的域名
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# 設定資料夾路徑
base_dir = Path(__file__).resolve().parent
static_dir = base_dir / "static"
html_dir = static_dir / "html"
css_dir = static_dir / "css"
js_dir = static_dir / "js"

# 掛載靜態資源
app.mount("/css", StaticFiles(directory=css_dir), name="css")
app.mount("/js", StaticFiles(directory=js_dir), name="js")

# Jinja2 Templates 設定
templates = Jinja2Templates(directory=str(html_dir))

# 掛載 API 路由
app.include_router(medicine_routes, prefix="/api", tags=["Medicine API"])
app.include_router(prescription_routes, prefix="/api", tags=["Prescription API"])

# 首頁與頁面路由
@app.get("/", response_class=HTMLResponse)
async def read_root(request: Request):
    return templates.TemplateResponse("Medicine.html", {"request": request})

@app.get("/Medicine.html", response_class=HTMLResponse)
async def render_medicine(request: Request):
    return templates.TemplateResponse("Medicine.html", {"request": request})

@app.get("/Prescription.html", response_class=HTMLResponse)
async def render_prescription(request: Request):
    return templates.TemplateResponse("Prescription.html", {"request": request})

@app.get("/doctor.html", response_class=HTMLResponse)
async def render_doctor(request: Request):
    return templates.TemplateResponse("doctor.html", {"request": request})

@app.get("/background.html", response_class=HTMLResponse)
async def render_background(request: Request):
    return templates.TemplateResponse("background.html", {"request": request})
