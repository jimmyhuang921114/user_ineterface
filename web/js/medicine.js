// 增強版藥物管理系統 JavaScript
class EnhancedMedicineManager {
    constructor() {
        this.apiBase = 'http://localhost:8000/api';
        this.hot = null;
        this.basicData = [];
        this.currentEditingMedicine = null;
        this.init();
    }

    async init() {
        await this.loadBasicMedicines();
        this.initBasicTable();
        this.initDetailedMedicineForm();
        this.setupEventListeners();
        await this.loadMedicineSelect();
    }

    // ==================== 基本庫存管理 ====================

    async loadBasicMedicines() {
        try {
            const response = await fetch(`${this.apiBase}/medicine/`);
            if (response.ok) {
                const medicines = await response.json();
                this.basicData = medicines.map(med => [
                    med.name,
                    med.amount,
                    med.usage_days,
                    med.position,
                    med.id
                ]);
            } else {
                console.error('Failed to load basic medicines:', response.statusText);
                this.basicData = [];
            }
        } catch (error) {
            console.error('Error loading basic medicines:', error);
            this.basicData = [];
        }
    }

    initBasicTable() {
        const container = document.getElementById('medicineTable');
        this.hot = new Handsontable(container, {
            data: this.basicData,
            colHeaders: ['藥品名稱', '數量', '使用天數', '儲存位置', '操作'],
            columns: [
                { type: 'text', placeholder: '輸入藥名' },
                { type: 'numeric', placeholder: '輸入數量' },
                { type: 'numeric', placeholder: '使用天數' },
                { type: 'text', placeholder: '儲位位置（如 A1-01）' },
                { type: 'text', readOnly: true, renderer: this.actionButtonRenderer.bind(this) }
            ],
            stretchH: 'all',
            rowHeaders: true,
            minSpareRows: 1,
            height: 400,
            licenseKey: 'non-commercial-and-evaluation',
            afterChange: this.handleCellChange.bind(this),
            contextMenu: ['remove_row'],
            afterRemoveRow: this.handleRowDelete.bind(this)
        });
    }

    actionButtonRenderer(instance, td, row, col, prop, value, cellProperties) {
        td.innerHTML = '';
        if (row < this.basicData.length && this.basicData[row].length >= 5) {
            const saveBtn = document.createElement('button');
            saveBtn.textContent = '保存';
            saveBtn.className = 'action-btn save-btn';
            saveBtn.onclick = () => this.saveBasicMedicine(row);

            const deleteBtn = document.createElement('button');
            deleteBtn.textContent = '刪除';
            deleteBtn.className = 'action-btn delete-btn';
            deleteBtn.onclick = () => this.deleteBasicMedicine(row);

            const viewBtn = document.createElement('button');
            viewBtn.textContent = '詳情';
            viewBtn.className = 'action-btn btn-info';
            viewBtn.onclick = () => this.viewMedicineDetails(this.basicData[row][0]);

            td.appendChild(saveBtn);
            td.appendChild(deleteBtn);
            td.appendChild(viewBtn);
        } else {
            const addBtn = document.createElement('button');
            addBtn.textContent = '新增';
            addBtn.className = 'action-btn add-btn';
            addBtn.onclick = () => this.addBasicMedicine(row);
            td.appendChild(addBtn);
        }
        return td;
    }

    async handleCellChange(changes, source) {
        if (!changes || source === 'loadData') return;
        
        for (const change of changes) {
            const [row, col, oldValue, newValue] = change;
            if (this.basicData[row] && this.basicData[row].length >= 5 && this.basicData[row][4]) {
                clearTimeout(this.saveTimeout);
                this.saveTimeout = setTimeout(() => {
                    this.saveBasicMedicine(row);
                }, 1000);
            }
        }
    }

    async addBasicMedicine(row) {
        const rowData = this.hot.getDataAtRow(row);
        const [name, amount, usage_days, position] = rowData;

        if (!name || !amount || !usage_days || !position) {
            this.showMessage('請填寫完整的藥物資訊', 'error');
            return;
        }

        try {
            const response = await fetch(`${this.apiBase}/medicine/`, {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({
                    name: name,
                    amount: parseInt(amount),
                    usage_days: parseInt(usage_days),
                    position: position
                })
            });

            if (response.ok) {
                const newMedicine = await response.json();
                this.basicData[row] = [name, amount, usage_days, position, newMedicine.id];
                this.hot.render();
                this.showMessage('藥物新增成功', 'success');
                await this.loadMedicineSelect(); // 更新選擇器
            } else {
                const error = await response.json();
                this.showMessage(`新增失敗: ${error.detail}`, 'error');
            }
        } catch (error) {
            console.error('Error adding medicine:', error);
            this.showMessage('新增失敗，請檢查網路連接', 'error');
        }
    }

    async saveBasicMedicine(row) {
        const rowData = this.hot.getDataAtRow(row);
        const [name, amount, usage_days, position, id] = rowData;

        if (!id) {
            await this.addBasicMedicine(row);
            return;
        }

        try {
            const response = await fetch(`${this.apiBase}/medicine/${id}`, {
                method: 'PUT',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({
                    name: name,
                    amount: parseInt(amount),
                    usage_days: parseInt(usage_days),
                    position: position
                })
            });

            if (response.ok) {
                this.showMessage('藥物更新成功', 'success');
            } else {
                const error = await response.json();
                this.showMessage(`更新失敗: ${error.detail}`, 'error');
            }
        } catch (error) {
            console.error('Error updating medicine:', error);
            this.showMessage('更新失敗，請檢查網路連接', 'error');
        }
    }

    async deleteBasicMedicine(row) {
        const rowData = this.hot.getDataAtRow(row);
        const id = rowData[4];

        if (!id) {
            this.hot.alter('remove_row', row);
            return;
        }

        if (!confirm('確定要刪除這個藥物嗎？')) {
            return;
        }

        try {
            const response = await fetch(`${this.apiBase}/medicine/${id}`, {
                method: 'DELETE'
            });

            if (response.ok) {
                this.hot.alter('remove_row', row);
                this.basicData.splice(row, 1);
                this.showMessage('藥物刪除成功', 'success');
                await this.loadMedicineSelect(); // 更新選擇器
            } else {
                const error = await response.json();
                this.showMessage(`刪除失敗: ${error.detail}`, 'error');
            }
        } catch (error) {
            console.error('Error deleting medicine:', error);
            this.showMessage('刪除失敗，請檢查網路連接', 'error');
        }
    }

    // ==================== 詳細藥物資訊管理 ====================

    initDetailedMedicineForm() {
        // 初始化表單和事件監聽器
        this.clearDetailedForm();
    }

    async loadMedicineSelect() {
        const select = document.getElementById('medicineSelect');
        select.innerHTML = '<option value="">-- 選擇藥物 --</option>';

        try {
            const response = await fetch(`${this.apiBase}/medicine/`);
            if (response.ok) {
                const medicines = await response.json();
                medicines.forEach(medicine => {
                    const option = document.createElement('option');
                    option.value = medicine.name;
                    option.textContent = `${medicine.name} (位置: ${medicine.position})`;
                    select.appendChild(option);
                });
            }
        } catch (error) {
            console.error('Error loading medicine select:', error);
        }
    }

    async loadDetailedMedicineInfo(medicineName) {
        if (!medicineName) {
            this.clearDetailedForm();
            return;
        }

        try {
            const response = await fetch(`${this.apiBase}/medicine/detailed/${encodeURIComponent(medicineName)}`);
            if (response.ok) {
                const data = await response.json();
                this.populateDetailedForm(medicineName, data);
                this.currentEditingMedicine = medicineName;
                document.getElementById('updateDetailedInfo').style.display = 'inline-block';
                document.getElementById('saveDetailedInfo').style.display = 'none';
            } else {
                // 沒有詳細資訊，準備新增
                this.clearDetailedForm();
                document.getElementById('medicineName').value = medicineName;
                this.currentEditingMedicine = null;
                document.getElementById('updateDetailedInfo').style.display = 'none';
                document.getElementById('saveDetailedInfo').style.display = 'inline-block';
                this.showMessage('此藥物尚無詳細資訊，請填寫後保存', 'info');
            }
        } catch (error) {
            console.error('Error loading detailed medicine info:', error);
            this.showMessage('載入詳細資訊失敗', 'error');
        }
    }

    populateDetailedForm(medicineName, data) {
        // 基本資訊
        document.getElementById('medicineName').value = data.基本資訊?.名稱 || medicineName;
        document.getElementById('dosage').value = data.基本資訊?.劑量 || '';
        document.getElementById('manufacturer').value = data.基本資訊?.廠商 || '';
        document.getElementById('usageMethod').value = data.基本資訊?.服用方式 || '';
        document.getElementById('unitDose').value = data.基本資訊?.單位劑量 || '';

        // 外觀
        document.getElementById('color').value = data.外觀?.顏色 || '';
        document.getElementById('shape').value = data.外觀?.形狀 || '';

        // 其他資訊
        document.getElementById('expiryDate').value = data.其他資訊?.有效日期 || '';
        document.getElementById('barcode').value = data.其他資訊?.條碼 || data.條碼 || '';
        document.getElementById('companyFullName').value = data.其他資訊?.公司全名 || '';
        document.getElementById('drugFullName').value = data.其他資訊?.藥物全名 || '';

        // 包裝編號
        document.getElementById('code1').value = data.包裝編號?.編號1 || '';
        document.getElementById('code2').value = data.包裝編號?.編號2 || '';
        document.getElementById('code3').value = data.包裝編號?.編號3 || '';

        // 大文字區域
        document.getElementById('indications').value = data.適應症 || '';
        document.getElementById('sideEffects').value = data.可能副作用 || data.可能的副作用 || '';
        document.getElementById('usage').value = data.使用說明 || '';
        document.getElementById('precautions').value = data.注意事項 || '';
        document.getElementById('pregnancyClass').value = data.懷孕分級 || '';
        document.getElementById('storageConditions').value = data.儲存條件 || '';
    }

    clearDetailedForm() {
        const inputs = [
            'medicineName', 'dosage', 'manufacturer', 'usageMethod', 'unitDose',
            'color', 'shape', 'expiryDate', 'barcode', 'companyFullName', 'drugFullName',
            'code1', 'code2', 'code3', 'indications', 'sideEffects', 'usage', 
            'precautions', 'pregnancyClass', 'storageConditions'
        ];

        inputs.forEach(id => {
            const element = document.getElementById(id);
            if (element) element.value = '';
        });

        this.currentEditingMedicine = null;
        document.getElementById('updateDetailedInfo').style.display = 'none';
        document.getElementById('saveDetailedInfo').style.display = 'inline-block';
    }

    getDetailedFormData() {
        return {
            基本資訊: {
                名稱: document.getElementById('medicineName').value,
                劑量: document.getElementById('dosage').value,
                廠商: document.getElementById('manufacturer').value,
                服用方式: document.getElementById('usageMethod').value,
                單位劑量: document.getElementById('unitDose').value
            },
            外觀: {
                顏色: document.getElementById('color').value,
                形狀: document.getElementById('shape').value
            },
            其他資訊: {
                有效日期: document.getElementById('expiryDate').value,
                條碼: document.getElementById('barcode').value,
                公司全名: document.getElementById('companyFullName').value,
                藥物全名: document.getElementById('drugFullName').value
            },
            包裝編號: {
                編號1: document.getElementById('code1').value,
                編號2: document.getElementById('code2').value,
                編號3: document.getElementById('code3').value
            },
            適應症: document.getElementById('indications').value,
            可能的副作用: document.getElementById('sideEffects').value,
            使用說明: document.getElementById('usage').value,
            注意事項: document.getElementById('precautions').value,
            懷孕分級: document.getElementById('pregnancyClass').value,
            儲存條件: document.getElementById('storageConditions').value
        };
    }

    async saveDetailedMedicineInfo() {
        const medicineName = document.getElementById('medicineName').value;
        if (!medicineName) {
            this.showMessage('請填寫藥物名稱', 'error');
            return;
        }

        const medicineData = this.getDetailedFormData();

        try {
            const response = await fetch(`${this.apiBase}/medicine/detailed/`, {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({
                    medicine_name: medicineName,
                    medicine_data: medicineData
                })
            });

            if (response.ok) {
                this.showMessage('詳細藥物資訊保存成功', 'success');
                this.currentEditingMedicine = medicineName;
                document.getElementById('updateDetailedInfo').style.display = 'inline-block';
                document.getElementById('saveDetailedInfo').style.display = 'none';
            } else {
                const error = await response.json();
                this.showMessage(`保存失敗: ${error.detail}`, 'error');
            }
        } catch (error) {
            console.error('Error saving detailed medicine info:', error);
            this.showMessage('保存失敗，請檢查網路連接', 'error');
        }
    }

    // ==================== 藥物資訊查看 ====================

    async searchDetailedMedicine(searchTerm, searchType = 'name') {
        if (!searchTerm.trim()) {
            this.showMessage('請輸入搜尋關鍵字', 'error');
            return;
        }

        try {
            let endpoint;
            switch (searchType) {
                case 'code':
                    endpoint = `/medicine/search/code/${encodeURIComponent(searchTerm)}`;
                    break;
                case 'name':
                default:
                    endpoint = `/medicine/integrated/${encodeURIComponent(searchTerm)}`;
                    break;
            }

            const response = await fetch(`${this.apiBase}${endpoint}`);
            if (response.ok) {
                const data = await response.json();
                this.displayMedicineInfo(data);
            } else {
                document.getElementById('medicineInfoDisplay').innerHTML = 
                    '<p class="placeholder-text">找不到相關藥物資訊</p>';
                this.showMessage('找不到相關藥物', 'error');
            }
        } catch (error) {
            console.error('Error searching medicine:', error);
            this.showMessage('搜尋失敗', 'error');
        }
    }

    displayMedicineInfo(data) {
        const display = document.getElementById('medicineInfoDisplay');
        
        if (data.medicine_name) {
            // 單一藥物的整合資訊
            display.innerHTML = this.formatIntegratedMedicineInfo(data);
        } else {
            // 多個搜尋結果
            display.innerHTML = this.formatSearchResults(data);
        }
    }

    formatIntegratedMedicineInfo(data) {
        let html = `<div class="medicine-info">
            <h3>${data.medicine_name}</h3>`;

        // 庫存資訊
        if (data.basic_info) {
            html += `<div class="info-section inventory-info">
                <h4>庫存資訊</h4>
                <div class="info-item">
                    <span class="info-label">數量：</span>
                    <span class="info-value">${data.basic_info.amount}</span>
                </div>
                <div class="info-item">
                    <span class="info-label">位置：</span>
                    <span class="info-value">${data.basic_info.position}</span>
                </div>
                <div class="info-item">
                    <span class="info-label">使用天數：</span>
                    <span class="info-value">${data.basic_info.usage_days}</span>
                </div>
            </div>`;
        }

        // 詳細資訊
        if (data.detailed_info) {
            const detailed = data.detailed_info;
            
            // 基本資訊
            if (detailed.基本資訊) {
                html += `<div class="info-section detailed-info">
                    <h4>基本資訊</h4>`;
                Object.entries(detailed.基本資訊).forEach(([key, value]) => {
                    if (value) {
                        html += `<div class="info-item">
                            <span class="info-label">${key}：</span>
                            <span class="info-value">${value}</span>
                        </div>`;
                    }
                });
                html += '</div>';
            }

            // 外觀
            if (detailed.外觀) {
                html += `<div class="info-section detailed-info">
                    <h4>外觀</h4>`;
                Object.entries(detailed.外觀).forEach(([key, value]) => {
                    if (value) {
                        html += `<div class="info-item">
                            <span class="info-label">${key}：</span>
                            <span class="info-value">${value}</span>
                        </div>`;
                    }
                });
                html += '</div>';
            }

            // 包裝編號
            if (detailed.包裝編號) {
                html += `<div class="info-section detailed-info">
                    <h4>包裝編號</h4>`;
                Object.entries(detailed.包裝編號).forEach(([key, value]) => {
                    if (value) {
                        html += `<div class="info-item">
                            <span class="info-label">${key}：</span>
                            <span class="info-value">${value}</span>
                        </div>`;
                    }
                });
                html += '</div>';
            }

            // 其他重要資訊
            const importantFields = [
                { key: '適應症', label: '適應症' },
                { key: '可能的副作用', label: '可能的副作用' },
                { key: '使用說明', label: '使用說明' },
                { key: '注意事項', label: '注意事項' },
                { key: '懷孕分級', label: '懷孕分級' },
                { key: '儲存條件', label: '儲存條件' }
            ];

            importantFields.forEach(field => {
                if (detailed[field.key]) {
                    html += `<div class="info-section detailed-info">
                        <h4>${field.label}</h4>
                        <div class="info-item">
                            <span class="info-value">${detailed[field.key]}</span>
                        </div>
                    </div>`;
                }
            });
        }

        html += '</div>';
        return html;
    }

    formatSearchResults(results) {
        let html = '<div class="medicine-info"><h3>搜尋結果</h3>';
        
        Object.entries(results).forEach(([name, data]) => {
            html += `<div class="info-section">
                <h4>${name}</h4>`;
            
            if (data.matched_code) {
                html += `<div class="info-item">
                    <span class="info-label">匹配編號：</span>
                    <span class="info-value">${data.matched_code.type} - ${data.matched_code.value}</span>
                </div>`;
            }
            
            html += `<button class="btn-info" onclick="medicineManager.viewMedicineDetails('${name}')">
                查看詳細資訊
            </button></div>`;
        });
        
        html += '</div>';
        return html;
    }

    async viewMedicineDetails(medicineName) {
        await this.searchDetailedMedicine(medicineName, 'name');
    }

    // ==================== JSON 操作 ====================

    showJSONPreview() {
        const medicineData = this.getDetailedFormData();
        const medicineName = document.getElementById('medicineName').value;
        
        if (!medicineName) {
            this.showMessage('請先填寫藥物名稱', 'error');
            return;
        }

        const jsonData = {
            藥物名稱: medicineName,
            ...medicineData
        };

        document.getElementById('jsonPreview').textContent = JSON.stringify(jsonData, null, 2);
        document.getElementById('jsonModal').style.display = 'block';
    }

    copyJSON() {
        const jsonText = document.getElementById('jsonPreview').textContent;
        navigator.clipboard.writeText(jsonText).then(() => {
            this.showMessage('JSON已複製到剪貼板', 'success');
        }).catch(err => {
            console.error('Failed to copy JSON:', err);
            this.showMessage('複製失敗', 'error');
        });
    }

    downloadJSON() {
        const jsonText = document.getElementById('jsonPreview').textContent;
        const medicineName = document.getElementById('medicineName').value || 'medicine';
        
        const blob = new Blob([jsonText], { type: 'application/json' });
        const url = URL.createObjectURL(blob);
        const a = document.createElement('a');
        a.href = url;
        a.download = `${medicineName}_${new Date().toISOString().split('T')[0]}.json`;
        document.body.appendChild(a);
        a.click();
        document.body.removeChild(a);
        URL.revokeObjectURL(url);
        
        this.showMessage('JSON檔案已下載', 'success');
    }

    async exportDetailedJSON() {
        try {
            const response = await fetch(`${this.apiBase}/export/medicines/integrated`);
            if (response.ok) {
                const data = await response.json();
                
                const blob = new Blob([JSON.stringify(data, null, 2)], { type: 'application/json' });
                const url = URL.createObjectURL(blob);
                const a = document.createElement('a');
                a.href = url;
                a.download = `detailed_medicines_${new Date().toISOString().split('T')[0]}.json`;
                document.body.appendChild(a);
                a.click();
                document.body.removeChild(a);
                URL.revokeObjectURL(url);
                
                this.showMessage('詳細藥物資訊已導出', 'success');
            } else {
                this.showMessage('導出失敗', 'error');
            }
        } catch (error) {
            console.error('Error exporting detailed JSON:', error);
            this.showMessage('導出失敗', 'error');
        }
    }

    // ==================== 事件處理 ====================

    setupEventListeners() {
        // 藥物選擇器
        document.getElementById('medicineSelect').addEventListener('change', (e) => {
            this.loadDetailedMedicineInfo(e.target.value);
        });

        // 詳細資訊按鈕
        document.getElementById('loadDetailedInfo').addEventListener('click', () => {
            const selectedMedicine = document.getElementById('medicineSelect').value;
            if (selectedMedicine) {
                this.loadDetailedMedicineInfo(selectedMedicine);
            } else {
                this.showMessage('請先選擇藥物', 'error');
            }
        });

        document.getElementById('clearForm').addEventListener('click', () => {
            this.clearDetailedForm();
        });

        document.getElementById('saveDetailedInfo').addEventListener('click', () => {
            this.saveDetailedMedicineInfo();
        });

        document.getElementById('updateDetailedInfo').addEventListener('click', () => {
            this.saveDetailedMedicineInfo();
        });

        document.getElementById('viewDetailedJSON').addEventListener('click', () => {
            this.showJSONPreview();
        });

        document.getElementById('exportDetailedJSON').addEventListener('click', () => {
            this.exportDetailedJSON();
        });

        // 搜尋功能
        document.getElementById('searchDetailedBtn').addEventListener('click', () => {
            const searchTerm = document.getElementById('searchMedicineInput').value;
            this.searchDetailedMedicine(searchTerm, 'name');
        });

        document.getElementById('searchByCodeBtn').addEventListener('click', () => {
            const searchTerm = document.getElementById('searchMedicineInput').value;
            this.searchDetailedMedicine(searchTerm, 'code');
        });

        document.getElementById('searchMedicineInput').addEventListener('keypress', (e) => {
            if (e.key === 'Enter') {
                this.searchDetailedMedicine(e.target.value, 'name');
            }
        });

        // 模態框
        document.querySelector('.close').addEventListener('click', () => {
            document.getElementById('jsonModal').style.display = 'none';
        });

        document.getElementById('copyJSON').addEventListener('click', () => {
            this.copyJSON();
        });

        document.getElementById('downloadJSON').addEventListener('click', () => {
            this.downloadJSON();
        });

        // 點擊模態框外部關閉
        window.addEventListener('click', (e) => {
            const modal = document.getElementById('jsonModal');
            if (e.target === modal) {
                modal.style.display = 'none';
            }
        });

        // 添加控制按鈕
        this.addControlButtons();
    }

    addControlButtons() {
        const header = document.querySelector('.header');
        if (!header) return;

        const buttonContainer = document.createElement('div');
        buttonContainer.className = 'control-buttons';

        // 重新載入按鈕
        const reloadBtn = this.createButton('重新載入資料', async () => {
            await this.loadBasicMedicines();
            this.hot.loadData(this.basicData);
            await this.loadMedicineSelect();
            this.showMessage('資料已重新載入', 'success');
        });

        // 導出基本資料按鈕
        const exportBasicBtn = this.createButton('導出基本資料', () => {
            this.exportBasicJSON();
        });

        buttonContainer.appendChild(reloadBtn);
        buttonContainer.appendChild(exportBasicBtn);
        header.appendChild(buttonContainer);
    }

    createButton(text, onClick) {
        const button = document.createElement('button');
        button.textContent = text;
        button.className = 'btn-secondary';
        button.addEventListener('click', onClick);
        return button;
    }

    exportBasicJSON() {
        const jsonData = this.basicData
            .filter(row => row.length >= 4 && row[0])
            .map(row => ({
                name: row[0],
                amount: row[1],
                usage_days: row[2],
                position: row[3],
                id: row[4] || null
            }));

        const blob = new Blob([JSON.stringify(jsonData, null, 2)], { type: 'application/json' });
        const url = URL.createObjectURL(blob);
        const a = document.createElement('a');
        a.href = url;
        a.download = `basic_medicines_${new Date().toISOString().split('T')[0]}.json`;
        document.body.appendChild(a);
        a.click();
        document.body.removeChild(a);
        URL.revokeObjectURL(url);
        
        this.showMessage('基本藥物資料已導出', 'success');
    }

    // ==================== 通用功能 ====================

    showMessage(message, type = 'info') {
        let messageDiv = document.getElementById('message');
        if (!messageDiv) {
            messageDiv = document.createElement('div');
            messageDiv.id = 'message';
            messageDiv.style.cssText = `
                position: fixed;
                top: 20px;
                right: 20px;
                padding: 15px 20px;
                border-radius: 5px;
                color: white;
                z-index: 3000;
                transition: opacity 0.3s;
                font-weight: 500;
                box-shadow: 0 4px 12px rgba(0,0,0,0.15);
            `;
            document.body.appendChild(messageDiv);
        }

        messageDiv.textContent = message;
        messageDiv.className = `message-${type}`;

        const colors = {
            success: '#27ae60',
            error: '#e74c3c',
            info: '#3498db'
        };
        messageDiv.style.backgroundColor = colors[type] || colors.info;
        messageDiv.style.opacity = '1';

        setTimeout(() => {
            if (messageDiv) {
                messageDiv.style.opacity = '0';
                setTimeout(() => {
                    if (messageDiv && messageDiv.parentNode) {
                        messageDiv.parentNode.removeChild(messageDiv);
                    }
                }, 300);
            }
        }, 3000);
    }

    async handleRowDelete(index, amount) {
        for (let i = 0; i < amount; i++) {
            const rowIndex = index + i;
            if (this.basicData[rowIndex] && this.basicData[rowIndex][4]) {
                await this.deleteMedicineById(this.basicData[rowIndex][4]);
            }
        }
        await this.loadMedicineSelect();
    }

    async deleteMedicineById(id) {
        try {
            await fetch(`${this.apiBase}/medicine/${id}`, { method: 'DELETE' });
        } catch (error) {
            console.error('Error deleting medicine by ID:', error);
        }
    }
}

// 全局變量和初始化
let medicineManager;

document.addEventListener("DOMContentLoaded", () => {
    medicineManager = new EnhancedMedicineManager();
}); 