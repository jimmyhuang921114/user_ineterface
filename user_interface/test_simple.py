from fastapi import FastAPI

app = FastAPI()

@app.get("/")
def read_root():
    return {"Hello": "World", "message": "醫院藥物管理系統API測試"}

@app.get("/test")
def test_endpoint():
    return {"status": "ok", "test": "成功"}

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)