#  -

##



### 1

```bash
# Python
python3 start_local.py

# Shell
chmod +x start_local.sh
./start_local.sh
```

### 2

```bash
#  user_interface
python3 enhanced_server.py

#
python3 final_server.py
```

### 3start.sh

 `start.sh` 17
```bash
cd /workspace/user_interface
```

```bash
cd "$(dirname "$0")"
```

##


```
~/user_ineterface/user_interface/
 enhanced_server.py
 final_server.py
 start_local.py      #
 start_local.sh      # Shell
 ...
```

##

1. ****
   ```bash
   ls -la *.py
   ```

2. **Python**
   ```bash
   python3 --version
   ```

3. ****
   ```bash
   pip3 install fastapi uvicorn pydantic requests
   ```

4. ****
   ```bash
   python3 enhanced_server.py
   ```

##


- http://localhost:8000
- http://localhost:8000/Medicine.html

##

1. ****
   ```bash
   lsof -i :8000
   ```

2. ****
   ```bash
   pkill -f uvicorn
   pkill -f python.*server
   ```

3. ****
   ```bash
   python3 enhanced_server.py
   #
   ```