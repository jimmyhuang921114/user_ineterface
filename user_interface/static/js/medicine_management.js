/**
 * 藥物管理系統 JavaScript
 * Medicine Management System JavaScript
 */

// 全域變數
let availableMedicines = [];
let selectedMedicine = null;

// API基礎URL
const API_BASE = '/api';

// 初始化
document.addEventListener('DOMContentLoaded', function() {
    initializeSystem();
    setupEventListeners();
    loadBasicMedicines();
});

/**
 * 系統初始化
 */
function initializeSystem() {
    console.log('💊 藥物管理系統初始化...');
}

/**
 * 設置事件監聽器
 */
function setupEventListeners() {
    // 基本藥物表單提交
    document.getElementById('basicMedicineForm').addEventListener('submit', handleBasicMedicineSubmit);
    
    // 詳細藥物表單提交
    document.getElementById('detailedMedicineForm').addEventListener('submit', handleDetailedMedicineSubmit);
}

/**
 * 標籤切換
 */
function switchTab(tabName) {
    // 移除所有活動狀態
    document.querySelectorAll('.tab').forEach(tab => tab.classList.remove('active'));
    document.querySelectorAll('.tab-content').forEach(content => content.classList.remove('active'));
    
    // 設置新的活動標籤
    event.target.classList.add('active');
    document.getElementById(tabName).classList.add('active');
    
    // 根據標籤執行特定操作
    if (tabName === 'detailed-medicine') {
        loadBasicMedicines();
    }
}

/**
 * 載入基本藥物清單
 */
async function loadBasicMedicines() {
    try {
        const response = await fetch(`${API_BASE}/medicine/basic`);
        const data = await response.json();
        availableMedicines = data.medicines || [];
        
        // 更新詳細藥物的選擇器
        updateMedicineSelector();
        
        console.log('✅ 已載入', availableMedicines.length, '種基本藥物');
    } catch (error) {
        console.error('載入基本藥物清單失敗:', error);
        availableMedicines = [];
        showStatus('❌ 載入藥物清單失敗', 'error', 'detailedMedicineStatus');
    }
}

/**
 * 更新藥物選擇器
 */
function updateMedicineSelector() {
    const select = document.getElementById('detailedMedicineSelect');
    select.innerHTML = '<option value="">-- 請選擇已存在的基本藥物 --</option>';
    
    availableMedicines.forEach(medicine => {
        const option = document.createElement('option');
        option.value = medicine.name;
        option.textContent = `${medicine.name} (庫存: ${medicine.amount}, 位置: ${medicine.position})`;
        
        // 如果庫存不足，標記為紅色
        if (medicine.amount < 10) {
            option.style.color = '#e74c3c';
            option.textContent += ' [庫存不足]';
        }
        
        select.appendChild(option);
    });
}

/**
 * 載入基本藥物資訊
 */
function loadBasicMedicineInfo() {
    const select = document.getElementById('detailedMedicineSelect');
    const medicineName = select.value;
    
    if (!medicineName) {
        hideBasicMedicineInfo();
        return;
    }
    
    selectedMedicine = availableMedicines.find(med => med.name === medicineName);
    
    if (selectedMedicine) {
        showBasicMedicineInfo(selectedMedicine);
    }
}

/**
 * 顯示基本藥物資訊
 */
function showBasicMedicineInfo(medicine) {
    document.getElementById('infoMedicineName').textContent = medicine.name;
    document.getElementById('infoMedicineAmount').textContent = medicine.amount;
    document.getElementById('infoMedicinePosition').textContent = medicine.position;
    document.getElementById('infoMedicinePrompt').textContent = medicine.prompt || '未設定';
    
    document.getElementById('basicMedicineInfo').classList.add('show');
}

/**
 * 隱藏基本藥物資訊
 */
function hideBasicMedicineInfo() {
    document.getElementById('basicMedicineInfo').classList.remove('show');
    selectedMedicine = null;
}

/**
 * 處理基本藥物表單提交
 */
async function handleBasicMedicineSubmit(event) {
    event.preventDefault();
    
    const formData = new FormData(event.target);
    
    try {
        // 收集基本資料
        const basicData = {
            name: formData.get('basicMedicineName'),
            amount: parseInt(formData.get('basicMedicineAmount')),
            position: formData.get('basicMedicinePosition'),
            usage_days: parseInt(formData.get('basicMedicineUsageDays')) || null,
            manufacturer: formData.get('basicMedicineManufacturer') || '',
            dosage: formData.get('basicMedicineDosage') || '',
            prompt: formData.get('basicMedicinePrompt') || ''
        };
        
        // 發送請求
        const response = await fetch(`${API_BASE}/medicine/basic`, {
            method: 'POST',
            headers: {
                'Content-Type': 'application/json',
            },
            body: JSON.stringify(basicData)
        });
        
        const result = await response.json();
        
        if (response.ok) {
            showStatus('✅ 基本藥物資料已成功保存', 'success', 'basicMedicineStatus');
            document.getElementById('basicMedicineForm').reset();
            
            // 重新載入藥物清單
            await loadBasicMedicines();
        } else {
            throw new Error(result.detail || '保存失敗');
        }
        
    } catch (error) {
        showStatus('❌ 錯誤: ' + error.message, 'error', 'basicMedicineStatus');
    }
}

/**
 * 處理詳細藥物表單提交
 */
async function handleDetailedMedicineSubmit(event) {
    event.preventDefault();
    
    if (!selectedMedicine) {
        showStatus('❌ 請先選擇要添加詳細資料的基本藥物', 'error', 'detailedMedicineStatus');
        return;
    }
    
    const formData = new FormData(event.target);
    
    try {
        // 收集詳細資料
        const detailedData = {
            medicine_name: selectedMedicine.name,
            description: formData.get('detailedMedicineDescription') || '',
            ingredient: formData.get('detailedMedicineIngredient') || '',
            category: formData.get('detailedMedicineCategory') || '',
            usage_method: formData.get('detailedMedicineUsageMethod') || '',
            unit_dose: formData.get('detailedMedicineUnitDose') || '',
            side_effects: formData.get('detailedMedicineSideEffects') || '',
            storage_conditions: formData.get('detailedMedicineStorageConditions') || '',
            expiry_date: formData.get('detailedMedicineExpiryDate') || '',
            barcode: formData.get('detailedMedicineBarcode') || '',
            appearance_type: formData.get('detailedMedicineAppearance') || '',
            notes: formData.get('detailedMedicineNotes') || ''
        };
        
        // 發送請求
        const response = await fetch(`${API_BASE}/medicine/detailed`, {
            method: 'POST',
            headers: {
                'Content-Type': 'application/json',
            },
            body: JSON.stringify(detailedData)
        });
        
        const result = await response.json();
        
        if (response.ok) {
            showStatus(`✅ ${selectedMedicine.name} 的詳細資料已成功保存`, 'success', 'detailedMedicineStatus');
            document.getElementById('detailedMedicineForm').reset();
            hideBasicMedicineInfo();
            
            // 重置選擇器
            document.getElementById('detailedMedicineSelect').value = '';
        } else {
            throw new Error(result.detail || '保存失敗');
        }
        
    } catch (error) {
        showStatus('❌ 錯誤: ' + error.message, 'error', 'detailedMedicineStatus');
    }
}

/**
 * 顯示狀態訊息
 */
function showStatus(message, type = 'success', elementId) {
    const statusDiv = document.getElementById(elementId);
    statusDiv.className = `status-message status-${type}`;
    statusDiv.textContent = message;
    statusDiv.style.display = 'block';
    
    // 5秒後自動隱藏
    setTimeout(() => {
        statusDiv.style.display = 'none';
    }, 5000);
}

/**
 * 驗證表單資料
 */
function validateForm(formData, requiredFields) {
    for (const field of requiredFields) {
        if (!formData.get(field) || formData.get(field).trim() === '') {
            return `${field} 為必填欄位`;
        }
    }
    return null;
}

/**
 * 格式化日期時間
 */
function formatDateTime(date) {
    return date.toLocaleString('zh-TW', {
        year: 'numeric',
        month: '2-digit',
        day: '2-digit',
        hour: '2-digit',
        minute: '2-digit'
    });
}