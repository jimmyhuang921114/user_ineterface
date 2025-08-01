# 醫院藥物管理系統 - 架構圖說明

## 🏛️ 系統整體架構圖

```mermaid
graph TB
    subgraph "用戶層 (User Layer)"
        USER[👤 醫生/護士/管理員]
    end
    
    subgraph "表現層 (Presentation Layer)"
        WEB[🌐 Web Browser]
        HTML[📄 HTML Pages]
        CSS[🎨 CSS Styles]
        JS[⚡ JavaScript]
    end
    
    subgraph "應用服務層 (Application Service Layer)"
        MAIN[🚀 main.py<br/>系統入口]
        FIXED[🔧 fixed_server.py<br/>FastAPI核心]
        CORS[🔒 CORS中間件]
        STATIC[📁 Static Files服務]
    end
    
    subgraph "API閘道層 (API Gateway Layer)"
        API_MED[💊 medicine_api.py<br/>藥物管理API]
        API_REC[📋 medical_record_api.py<br/>病歷API]
        API_PRE[📝 prescription_api.py<br/>處方籤API]
    end
    
    subgraph "路由層 (Routing Layer)"
        ROUTE_MED[🛤️ routes_medicine.py<br/>藥物路由]
        ROUTE_PRE[🛤️ routes_prescription.py<br/>處方籤路由]
    end
    
    subgraph "業務邏輯層 (Business Logic Layer)"
        CRUD_MED[⚙️ crud_medicine.py<br/>藥物CRUD]
        CRUD_PRE[⚙️ crud_prescription.py<br/>處方籤CRUD]
    end
    
    subgraph "數據訪問層 (Data Access Layer)"
        SCHEMA_MED[📊 medicine_schema.py<br/>藥物模式]
        SCHEMA_PRE[📊 prescription_schemas.py<br/>處方籤模式]
        MODEL_MED[🗃️ medicine_models.py<br/>藥物模型]
        MODEL_PRE[🗃️ prescription_models.py<br/>處方籤模型]
    end
    
    subgraph "數據存儲層 (Data Storage Layer)"
        DB[(🗄️ SQLite Database)]
        JSON_MED[📄 medicines.json]
        JSON_DET[📄 detailed_medicines.json]
        JSON_PRE[📄 prescriptions.json]
    end
    
    USER --> WEB
    WEB --> HTML
    WEB --> CSS
    WEB --> JS
    
    HTML --> MAIN
    JS --> API_MED
    JS --> API_REC
    JS --> API_PRE
    
    MAIN --> FIXED
    FIXED --> CORS
    FIXED --> STATIC
    FIXED --> API_MED
    FIXED --> API_REC
    FIXED --> API_PRE
    
    API_MED --> ROUTE_MED
    API_PRE --> ROUTE_PRE
    API_REC --> ROUTE_MED
    
    ROUTE_MED --> CRUD_MED
    ROUTE_PRE --> CRUD_PRE
    
    CRUD_MED --> SCHEMA_MED
    CRUD_PRE --> SCHEMA_PRE
    CRUD_MED --> MODEL_MED
    CRUD_PRE --> MODEL_PRE
    
    MODEL_MED --> DB
    MODEL_PRE --> DB
    CRUD_MED --> JSON_MED
    CRUD_MED --> JSON_DET
    CRUD_PRE --> JSON_PRE
    
    STATIC --> HTML
    STATIC --> CSS
    STATIC --> JS
```

## 📦 包依賴關係圖

```mermaid
graph LR
    subgraph "前端包 (Frontend Packages)"
        FE_HTML[static/html/]
        FE_CSS[static/css/]
        FE_JS[static/js/]
    end
    
    subgraph "API包 (API Packages)"
        API[api/]
        ROUTE[route/]
    end
    
    subgraph "核心業務包 (Core Business Packages)"
        SERVICE[services/]
        MODEL[model/]
        SCHEMA[schemas/]
    end
    
    subgraph "數據包 (Data Packages)"
        DATA[data/]
    end
    
    subgraph "主應用包 (Main Application)"
        MAIN_APP[main.py + fixed_server.py]
    end
    
    FE_JS --> API
    API --> ROUTE
    API --> SERVICE
    ROUTE --> SERVICE
    SERVICE --> MODEL
    SERVICE --> SCHEMA
    MODEL --> DATA
    MAIN_APP --> API
    MAIN_APP --> FE_HTML
    MAIN_APP --> FE_CSS
    MAIN_APP --> FE_JS
    
    style API fill:#e1f5fe
    style SERVICE fill:#f3e5f5
    style MODEL fill:#e8f5e8
    style MAIN_APP fill:#fff3e0
```

## 🔄 數據流向圖

```mermaid
flowchart TD
    subgraph "前端交互流程"
        A[用戶操作] --> B[前端JavaScript]
        B --> C[AJAX請求]
    end
    
    subgraph "API處理流程"
        C --> D[API路由接收]
        D --> E[數據驗證]
        E --> F[調用業務邏輯]
    end
    
    subgraph "業務處理流程"
        F --> G[CRUD操作]
        G --> H[數據模型轉換]
        H --> I[數據庫操作]
    end
    
    subgraph "響應返回流程"
        I --> J[結果包裝]
        J --> K[JSON序列化]
        K --> L[HTTP響應]
        L --> M[前端接收]
        M --> N[界面更新]
    end
    
    style A fill:#ffebee
    style F fill:#e3f2fd
    style I fill:#e8f5e8
    style N fill:#fff3e0
```

## 📋 模組職責分工圖

```mermaid
mindmap
    root((醫院藥物管理系統))
        前端層
            用戶界面
                藥物管理頁面
                處方籤管理頁面
                醫生開立處方籤頁面
            交互邏輯
                表格操作
                搜尋功能
                數據導出
        應用層
            主應用
                系統啟動
                配置管理
            服務器
                API服務
                靜態文件服務
                中間件配置
        API層
            藥物API
                基本CRUD
                搜尋功能
                詳細資訊管理
            處方籤API
                處方籤CRUD
                狀態管理
            病歷API
                患者管理
                病歷記錄
        業務層
            藥物服務
                庫存管理
                搜尋邏輯
                數據驗證
            處方籤服務
                處方籤邏輯
                狀態更新
        數據層
            模型定義
                藥物模型
                處方籤模型
                關聯關係
            數據存儲
                SQLite數據庫
                JSON文件
                備份機制
```

## 🎯 分層架構詳解

### 1. 表現層 (Presentation Layer)
```
📱 用戶界面層
├── 🌐 Web瀏覽器
├── 📄 HTML模板 (static/html/)
├── 🎨 CSS樣式 (static/css/)
└── ⚡ JavaScript邏輯 (static/js/)
```
**職責**: 用戶交互、數據展示、前端驗證

### 2. 應用服務層 (Application Service Layer)
```
🚀 應用程式層
├── 🔧 main.py (系統入口)
├── 🛠️ fixed_server.py (FastAPI核心)
├── 🔒 中間件配置 (CORS等)
└── 📁 靜態文件服務
```
**職責**: 應用程式啟動、全局配置、請求分發

### 3. API閘道層 (API Gateway Layer)
```
🌐 API閘道層
├── 💊 medicine_api.py (藥物管理)
├── 📋 medical_record_api.py (病歷管理)
└── 📝 prescription_api.py (處方籤管理)
```
**職責**: API端點定義、請求路由、響應格式化

### 4. 業務邏輯層 (Business Logic Layer)
```
⚙️ 業務邏輯層
├── 🔨 crud_medicine.py (藥物業務邏輯)
└── 🔨 crud_prescription.py (處方籤業務邏輯)
```
**職責**: 核心業務邏輯、數據處理、業務規則驗證

### 5. 數據訪問層 (Data Access Layer)
```
📊 數據訪問層
├── 🗃️ models/ (數據模型)
│   ├── medicine_models.py
│   └── prescription_models.py
└── 📋 schemas/ (API模式)
    ├── medicine_schema.py
    └── prescription_schemas.py
```
**職責**: 數據模型定義、ORM映射、數據驗證

### 6. 數據存儲層 (Data Storage Layer)
```
🗄️ 數據存儲層
├── 📘 SQLite資料庫
└── 📄 JSON文件存儲
    ├── medicines.json
    ├── detailed_medicines.json
    └── prescriptions.json
```
**職責**: 數據持久化、數據備份、數據恢復

## 🔗 包間通信協議

### API調用流程
```mermaid
sequenceDiagram
    participant FE as 前端界面
    participant API as API層
    participant BL as 業務邏輯層
    participant DA as 數據訪問層
    participant DB as 數據庫
    
    FE->>API: HTTP請求
    API->>API: 請求驗證
    API->>BL: 調用業務方法
    BL->>BL: 業務邏輯處理
    BL->>DA: 數據操作請求
    DA->>DB: SQL查詢/更新
    DB-->>DA: 查詢結果
    DA-->>BL: 數據對象
    BL-->>API: 處理結果
    API-->>FE: JSON響應
```

### 錯誤處理流程
```mermaid
flowchart TD
    START[請求開始] --> VALIDATE[數據驗證]
    VALIDATE -->|驗證失敗| ERROR1[400 Bad Request]
    VALIDATE -->|驗證成功| BUSINESS[業務邏輯處理]
    BUSINESS -->|業務錯誤| ERROR2[422 Unprocessable Entity]
    BUSINESS -->|處理成功| DATABASE[數據庫操作]
    DATABASE -->|數據庫錯誤| ERROR3[500 Internal Server Error]
    DATABASE -->|操作成功| SUCCESS[200 OK]
    
    ERROR1 --> RESPONSE[錯誤響應]
    ERROR2 --> RESPONSE
    ERROR3 --> RESPONSE
    SUCCESS --> RESPONSE[成功響應]
    RESPONSE --> END[請求結束]
    
    style ERROR1 fill:#ffcdd2
    style ERROR2 fill:#ffcdd2
    style ERROR3 fill:#ffcdd2
    style SUCCESS fill:#c8e6c9
```

## 📈 擴展性設計

### 水平擴展點
```mermaid
graph LR
    subgraph "可擴展的API模組"
        API1[藥物API]
        API2[處方籤API]
        API3[患者API]
        API4[🔮 庫存API]
        API5[🔮 報表API]
        API6[🔮 通知API]
    end
    
    subgraph "可擴展的業務模組"
        BL1[藥物服務]
        BL2[處方籤服務]
        BL3[患者服務]
        BL4[🔮 庫存服務]
        BL5[🔮 報表服務]
        BL6[🔮 通知服務]
    end
    
    API1 --> BL1
    API2 --> BL2
    API3 --> BL3
    API4 --> BL4
    API5 --> BL5
    API6 --> BL6
    
    style API4 stroke-dasharray: 5 5
    style API5 stroke-dasharray: 5 5
    style API6 stroke-dasharray: 5 5
    style BL4 stroke-dasharray: 5 5
    style BL5 stroke-dasharray: 5 5
    style BL6 stroke-dasharray: 5 5
```

### 垂直擴展點
```mermaid
graph TD
    A[當前架構] --> B[微服務架構]
    A --> C[容器化部署]
    A --> D[分布式數據庫]
    A --> E[負載均衡]
    A --> F[API閘道]
    A --> G[服務發現]
    
    style B stroke-dasharray: 5 5
    style C stroke-dasharray: 5 5
    style D stroke-dasharray: 5 5
    style E stroke-dasharray: 5 5
    style F stroke-dasharray: 5 5
    style G stroke-dasharray: 5 5
```

## 🎯 總結

這個架構設計具有以下特點：

### ✅ 優勢
1. **分層清晰**: 每層職責明確，易於維護
2. **低耦合**: 層間依賴關係簡單，易於測試
3. **高內聚**: 同層模組功能相關性強
4. **可擴展**: 易於添加新功能和新模組
5. **標準化**: 遵循RESTful API設計原則

### 🔧 技術亮點
1. **FastAPI**: 現代Python Web框架
2. **SQLModel**: 類型安全的ORM
3. **Pydantic**: 數據驗證和序列化
4. **分層架構**: 清晰的職責分離
5. **模組化設計**: 易於維護和擴展

### 🚀 未來擴展方向
1. **微服務架構**: 將各模組拆分為獨立服務
2. **容器化部署**: 使用Docker進行部署
3. **API閘道**: 統一API管理和安全控制
4. **分布式存儲**: 支持更大規模的數據處理
5. **實時通知**: 添加WebSocket支持實時更新