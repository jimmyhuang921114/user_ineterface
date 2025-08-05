/**
 * 醫生工作台 JavaScript
 * Doctor Workstation JavaScript
 */

// 全域變數
let availableMedicines = [];
let medicineCounter = 1;

// API基礎URL
const API_BASE = '/api';

// 初始化
document.addEventListener('DOMContentLoaded', function() {
    initializeSystem();
    setupEventListeners();
    loadAvailableMedicines();
    generateAutoFields();
});

/**
 * 系統初始化
 */
function initializeSystem() {
    console.log('👨‍⚕️ 醫生工作台初始化...');
    
    // 初始化處方時間
    updatePrescriptionTime();
    
    // 初始化第一個藥物項目
    addMedicineItem();
}

/**
 * 設置事件監聽器
 */
function setupEventListeners() {
    // 統一藥物表單提交
    document.getElementById('unifiedMedicineForm').addEventListener('submit', handleMedicineSubmit);
    
    // 處方籤表單提交
    document.getElementById('prescriptionForm').addEventListener('submit', handlePrescriptionSubmit);
    
    // 身份證號輸入時自動生成病患編號
    document.getElementById('patientIdCard').addEventListener('input', generatePatientNumber);
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
    if (tabName === 'prescription') {
        updatePrescriptionTime();
        loadAvailableMedicines();
    }
}

/**
 * 載入可用藥物清單
 */
async function loadAvailableMedicines() {
    try {
        const response = await fetch(`${API_BASE}/medicine/basic`);
        const data = await response.json();
        availableMedicines = data.medicines || [];
        
        // 更新所有藥物下拉選單
        updateMedicineSelects();
        
        console.log('✅ 已載入', availableMedicines.length, '種可用藥物');
    } catch (error) {
        console.error('載入藥物清單失敗:', error);
        availableMedicines = [];
    }
}

/**
 * 更新藥物下拉選單
 */
function updateMedicineSelects() {
    const selects = document.querySelectorAll('.medicine-select select');
    
    selects.forEach(select => {
        const currentValue = select.value;
        select.innerHTML = '<option value="">請選擇藥物</option>';
        
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
        
        // 恢復之前的選擇
        if (currentValue) {
            select.value = currentValue;
        }
    });
}

/**
 * 處理統一藥物表單提交
 */
async function handleMedicineSubmit(event) {
    event.preventDefault();
    
    const formData = new FormData(event.target);
    
    try {
        // 收集基本資料
        const basicData = {
            name: formData.get('medicineName'),
            amount: parseInt(formData.get('medicineAmount')),
            position: formData.get('medicinePosition'),
            usage_days: parseInt(formData.get('medicineUsageDays')) || null,
            manufacturer: formData.get('medicineManufacturer') || '',
            dosage: formData.get('medicineDosage') || ''
        };
        
        // 收集詳細資料
        const detailedData = {
            description: formData.get('medicineDescription') || '',
            ingredient: formData.get('medicineIngredient') || '',
            category: formData.get('medicineCategory') || '',
            usage_method: formData.get('medicineUsageMethod') || '',
            unit_dose: formData.get('medicineUnitDose') || '',
            side_effects: formData.get('medicineSideEffects') || '',
            storage_conditions: formData.get('medicineStorageConditions') || '',
            expiry_date: formData.get('medicineExpiryDate') || '',
            barcode: formData.get('medicineBarcode') || '',
            appearance_type: formData.get('medicineAppearance') || '',
            notes: formData.get('medicineNotes') || ''
        };
        
        // 檢查是否有詳細資料
        const hasDetailedData = Object.values(detailedData).some(value => value !== '');
        
        // 發送請求
        const response = await fetch(`${API_BASE}/medicine/unified`, {
            method: 'POST',
            headers: {
                'Content-Type': 'application/json',
            },
            body: JSON.stringify({
                basic_data: basicData,
                detailed_data: hasDetailedData ? detailedData : null
            })
        });
        
        const result = await response.json();
        
        if (response.ok) {
            showStatus('✅ ' + result.message, 'success', 'medicineStatus');
            document.getElementById('unifiedMedicineForm').reset();
            
            // 重新載入藥物清單供處方籤使用
            await loadAvailableMedicines();
        } else {
            throw new Error(result.detail || '保存失敗');
        }
        
    } catch (error) {
        showStatus('❌ 錯誤: ' + error.message, 'error', 'medicineStatus');
    }
}

/**
 * 處理處方籤表單提交
 */
async function handlePrescriptionSubmit(event) {
    event.preventDefault();
    
    try {
        // 收集病患資訊
        const patientName = document.getElementById('patientName').value.trim();
        const patientIdCard = document.getElementById('patientIdCard').value.trim();
        const patientNumber = document.getElementById('patientNumber').textContent;
        
        if (!patientName || !patientIdCard) {
            throw new Error('請填寫完整的病患資訊');
        }
        
        if (patientIdCard.length !== 10) {
            throw new Error('身份證號必須為10位數');
        }
        
        // 收集處方用藥
        const medicines = [];
        const medicineItems = document.querySelectorAll('.medicine-item');
        
        medicineItems.forEach(item => {
            const medicineName = item.querySelector('.medicine-select select').value;
            const dosage = item.querySelector('input[placeholder*="劑量"]').value.trim();
            const frequency = item.querySelector('input[placeholder*="頻率"]').value.trim();
            const duration = item.querySelector('input[placeholder*="天數"]').value.trim();
            const notes = item.querySelector('input[placeholder*="備註"]').value.trim();
            
            if (medicineName && dosage && frequency && duration) {
                medicines.push([
                    medicineName,
                    dosage,
                    frequency,
                    duration,
                    notes || ''
                ]);
            }
        });
        
        if (medicines.length === 0) {
            throw new Error('請至少選擇一種處方用藥');
        }
        
        // 構建處方籤資料
        const prescriptionData = {
            patient_name: patientName,
            patient_id: patientNumber,
            doctor_name: '系統醫生', // 預設醫生名稱
            medicines: medicines,
            created_at: new Date().toISOString()
        };
        
        // 發送請求
        const response = await fetch(`${API_BASE}/prescription/`, {
            method: 'POST',
            headers: {
                'Content-Type': 'application/json',
            },
            body: JSON.stringify(prescriptionData)
        });
        
        const result = await response.json();
        
        if (response.ok) {
            showStatus('✅ 處方籤已成功開立', 'success', 'prescriptionStatus');
            
            // 重置表單
            document.getElementById('prescriptionForm').reset();
            generateAutoFields();
            
            // 清空藥物列表並重新添加一個
            document.getElementById('medicineList').innerHTML = '';
            addMedicineItem();
            
        } else {
            throw new Error(result.detail || '開立處方籤失敗');
        }
        
    } catch (error) {
        showStatus('❌ 錯誤: ' + error.message, 'error', 'prescriptionStatus');
    }
}

/**
 * 生成自動欄位
 */
function generateAutoFields() {
    updatePrescriptionTime();
    // 病患編號會在輸入身份證號時自動生成
    document.getElementById('patientNumber').textContent = '請先輸入身份證號';
}

/**
 * 更新處方時間
 */
function updatePrescriptionTime() {
    const now = new Date();
    const timeString = now.toLocaleString('zh-TW', {
        year: 'numeric',
        month: '2-digit',
        day: '2-digit',
        hour: '2-digit',
        minute: '2-digit',
        second: '2-digit'
    });
    document.getElementById('prescriptionTime').textContent = timeString;
}

/**
 * 根據身份證號生成病患編號
 */
function generatePatientNumber() {
    const idCard = document.getElementById('patientIdCard').value.trim();
    
    if (idCard.length >= 4) {
        // 使用身份證號後4位 + 時間戳生成唯一編號
        const timestamp = Date.now().toString().slice(-6);
        const lastFour = idCard.slice(-4);
        const patientNumber = `P${lastFour}${timestamp}`;
        
        document.getElementById('patientNumber').textContent = patientNumber;
    } else {
        document.getElementById('patientNumber').textContent = '請先輸入身份證號';
    }
}

/**
 * 新增藥物項目
 */
function addMedicineItem() {
    const medicineList = document.getElementById('medicineList');
    const itemId = `medicine-${medicineCounter++}`;
    
    const medicineItem = document.createElement('div');
    medicineItem.className = 'medicine-item';
    medicineItem.id = itemId;
    
    medicineItem.innerHTML = `
        <div class="medicine-select">
            <label>藥物名稱 *</label>
            <select required onchange="updateMedicineInfo(this)">
                <option value="">請選擇藥物</option>
            </select>
        </div>
        <div class="medicine-input">
            <label>劑量 *</label>
            <input type="text" placeholder="例：500mg" required>
        </div>
        <div class="medicine-input">
            <label>頻率 *</label>
            <input type="text" placeholder="例：每日三次" required>
        </div>
        <div class="medicine-input">
            <label>天數 *</label>
            <input type="number" placeholder="例：7" min="1" required>
        </div>
        <div class="medicine-input">
            <label>備註</label>
            <input type="text" placeholder="特殊說明">
        </div>
        <button type="button" class="remove-medicine" onclick="removeMedicineItem('${itemId}')" title="移除此藥物">
            ✕
        </button>
    `;
    
    medicineList.appendChild(medicineItem);
    
    // 更新新增的下拉選單
    updateMedicineSelects();
    
    console.log('✅ 已新增藥物項目:', itemId);
}

/**
 * 移除藥物項目
 */
function removeMedicineItem(itemId) {
    const item = document.getElementById(itemId);
    if (item) {
        item.remove();
        console.log('🗑️ 已移除藥物項目:', itemId);
        
        // 如果沒有藥物項目了，至少保留一個
        const medicineList = document.getElementById('medicineList');
        if (medicineList.children.length === 0) {
            addMedicineItem();
        }
    }
}

/**
 * 當選擇藥物時更新藥物資訊
 */
function updateMedicineInfo(selectElement) {
    const selectedMedicine = availableMedicines.find(med => med.name === selectElement.value);
    
    if (selectedMedicine) {
        const medicineItem = selectElement.closest('.medicine-item');
        
        // 自動填入建議劑量
        if (selectedMedicine.dosage) {
            const dosageInput = medicineItem.querySelector('input[placeholder*="劑量"]');
            if (dosageInput && !dosageInput.value) {
                dosageInput.value = selectedMedicine.dosage;
            }
        }
        
        // 自動填入建議使用天數
        if (selectedMedicine.usage_days) {
            const durationInput = medicineItem.querySelector('input[placeholder*="天數"]');
            if (durationInput && !durationInput.value) {
                durationInput.value = selectedMedicine.usage_days;
            }
        }
        
        // 檢查庫存警告
        if (selectedMedicine.amount < 10) {
            selectElement.style.borderColor = '#e74c3c';
            selectElement.title = `警告：此藥物庫存不足 (剩餘: ${selectedMedicine.amount})`;
        } else {
            selectElement.style.borderColor = '#27ae60';
            selectElement.title = `庫存充足 (剩餘: ${selectedMedicine.amount})`;
        }
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
    
    // 3秒後自動隱藏
    setTimeout(() => {
        statusDiv.style.display = 'none';
    }, 3000);
}

/**
 * 驗證身份證號格式
 */
function validateIdCard(idCard) {
    // 台灣身份證號格式驗證
    const pattern = /^[A-Z][12]\d{8}$/;
    return pattern.test(idCard);
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

// 定期更新處方時間 (每分鐘)
setInterval(updatePrescriptionTime, 60000);