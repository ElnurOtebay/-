import adsk.core, adsk.fusion, adsk.cam, traceback

def run(context):
    ui = None
    try:
        # Получение объектов приложения и пользовательского интерфейса
        app = adsk.core.Application.get()
        ui = app.userInterface
        product = app.activeProduct
        design = adsk.fusion.Design.cast(product)
        
        # Проверка, что активен режим проектирования
        if not design:
            ui.messageBox('Необходимо открыть документ Fusion и перейти в режим Design')
            return
            
        # Получение активного компонента для работы
        rootComp = design.rootComponent
        
        # Сохраняем позицию таймлайна для отката при ошибке
        startIndex = design.timeline.markerPosition
        
        # ЭТАП 1: ВЫБОР ТРУБ
        # Запрашиваем у пользователя выбор режущей трубы (инструмента)
        ui.messageBox('Сначала выберите трубу, которая будет использоваться как режущий инструмент')
        cuttingBodySelect = ui.selectEntity('Выберите РЕЖУЩУЮ трубу (которая будет использоваться как инструмент)', 'SolidBodies')
        if not cuttingBodySelect:
            return  # Пользователь отменил выбор
        
        cuttingBody = adsk.fusion.BRepBody.cast(cuttingBodySelect.entity)
        
        # Запрашиваем у пользователя выбор целевой трубы (которую нужно обрезать)
        ui.messageBox('Теперь выберите трубу, которая будет обрезана')
        targetBodySelect = ui.selectEntity('Выберите ЦЕЛЕВУЮ трубу (которая будет обрезана)', 'SolidBodies')
        if not targetBodySelect:
            return  # Пользователь отменил выбор
        
        targetBody = adsk.fusion.BRepBody.cast(targetBodySelect.entity)
        
        # ЭТАП 2: АНАЛИЗ ЦЕЛЕВОЙ ТРУБЫ
        # Находим внутреннюю поверхность целевой трубы
        innerFace = None
        smallestRadius = float('inf')
        
        # Перебираем все грани целевой трубы
        for face in targetBody.faces:
            # Проверяем, является ли грань цилиндрической
            if face.geometry.objectType == adsk.core.Cylinder.classType():
                # Получаем цилиндр и его радиус
                cylinder = adsk.core.Cylinder.cast(face.geometry)
                radius = cylinder.radius
                
                # Если это самая маленькая цилиндрическая поверхность, значит это внутренняя поверхность
                if radius < smallestRadius:
                    smallestRadius = radius
                    innerFace = face
        
        if not innerFace:
            ui.messageBox('Не удалось найти внутреннюю поверхность трубы. Убедитесь, что выбрана цилиндрическая труба.')
            return
        
        # Сохраняем точку на внутренней поверхности для дальнейшего поиска
        pointOnInnerFace = innerFace.pointOnFace
        
        # ЭТАП 3: РАЗРЕЗАНИЕ ВНУТРЕННЕЙ ПОВЕРХНОСТИ
        # Создаем коллекцию с внутренней поверхностью трубы
        targetFaces = adsk.core.ObjectCollection.create()
        targetFaces.add(innerFace)
        
        # Получаем исходную площадь внутренней поверхности для последующего сравнения
        originalArea = innerFace.evaluator.area
        
        # Получаем функции разделения поверхности
        splitFaceFeats = rootComp.features.splitFaceFeatures
        
        # Создаем объект ввода для операции разделения поверхности
        splitFaceInput = splitFaceFeats.createInput(targetFaces, cuttingBody, True)
        
        # Устанавливаем тип разрезания как пересечение поверхностей
        splitFaceInput.setSurfaceIntersectionSplitType(True)
        
        # Выполняем операцию разрезания внутренней поверхности
        try:
            splitFaceResult = splitFaceFeats.add(splitFaceInput)
        except Exception as e:
            ui.messageBox(f'Не удалось разрезать внутреннюю поверхность трубы:\n{str(e)}\n'
                        f'Убедитесь, что трубы пересекаются.')
            return
        
        # ЭТАП 4: ПОИСК РАЗРЕЗАННЫХ ПОВЕРХНОСТЕЙ И ВЫБОР НУЖНОЙ
        # Находим все цилиндрические грани целевой трубы после разделения
        cylindricalFaces = []
        for face in targetBody.faces:
            if face.geometry.objectType == adsk.core.Cylinder.classType():
                cylinder = adsk.core.Cylinder.cast(face.geometry)
                # Проверяем, что это грань с тем же радиусом, что и исходная внутренняя поверхность
                if abs(cylinder.radius - smallestRadius) < 0.001:  # Допуск 0.001 см
                    cylindricalFaces.append(face)
        
        # Сортируем поверхности по площади (от большей к меньшей)
        cylindricalFaces.sort(key=lambda face: face.evaluator.area, reverse=True)
        
        # Выбираем самую большую поверхность
        if not cylindricalFaces:
            ui.messageBox('Не удалось найти внутренние поверхности после разделения.')
            return
        
        # Берем самую большую часть (основная часть трубы)
        largestFace = cylindricalFaces[0]
        
        # ЭТАП 5: СОЗДАНИЕ СМЕЩЕННОЙ ПОВЕРХНОСТИ (OFFSET)
        # Создаем коллекцию с самой большой поверхностью
        facesToOffset = adsk.core.ObjectCollection.create()
        facesToOffset.add(largestFace)
        
        # Получаем функции создания смещенных поверхностей
        offsetFeats = rootComp.features.offsetFeatures
        
        # Создаем минимальное смещение для создания отдельной поверхности (0.001 мм)
        offsetDistance = adsk.core.ValueInput.createByReal(0.001)
        
        # Создаем объект ввода для операции offset
        offsetInput = offsetFeats.createInput(facesToOffset, offsetDistance, adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
        
        # Отключаем выбор связанных поверхностей
        offsetInput.isChainSelection = False
        
        # Выполняем операцию создания смещенной поверхности
        try:
            offsetResult = offsetFeats.add(offsetInput)
        except Exception as e:
            ui.messageBox(f'Не удалось создать смещенную поверхность:\n{str(e)}')
            return
        
        # Используем findBRepUsingPoint для нахождения созданного поверхностного тела
        # на месте оригинальной поверхности
        offsetBodies = []
        bodyProxies = rootComp.findBRepUsingPoint(pointOnInnerFace, adsk.fusion.BRepEntityTypes.BRepBodyEntityType, 0.01, True)
        
        # Проверяем найденные тела и ищем то, которое не является ни режущим телом, ни целевым телом
        for bodyProxy in bodyProxies:
            body = adsk.fusion.BRepBody.cast(bodyProxy)
            if body != targetBody and body != cuttingBody and not body.isSolid:
                # Это должно быть поверхностное тело, созданное с помощью offset
                offsetBodies.append(body)
        
        if not offsetBodies:
            ui.messageBox('Не удалось найти созданное смещенное поверхностное тело.')
            return
        
        # Берем первое найденное смещенное тело
        offsetBody = offsetBodies[0]
        
        # ЭТАП 6: СОЗДАНИЕ ТВЕРДОГО ТЕЛА С ПОМОЩЬЮ THICKEN
        # Запрашиваем толщину стенки трубы для утолщения
        thicknessStr = ui.inputBox('Введите толщину стенки трубы (мм):', 'Толщина стенки', '2')[0]
        try:
            thickness = float(thicknessStr)
        except:
            ui.messageBox('Некорректное значение толщины. Будет использовано значение по умолчанию: 2 мм')
            thickness = 2.0
        
        # Создаем коллекцию из найденного поверхностного тела
        bodiesToThicken = adsk.core.ObjectCollection.create()
        bodiesToThicken.add(offsetBody)
        
        # Получаем функции утолщения поверхности
        thickenFeats = rootComp.features.thickenFeatures
        
        # Создаем отрицательное значение толщины (для направления внутрь)
        thicknessValue = adsk.core.ValueInput.createByReal(-thickness)
        
        # Создаем объект ввода для операции утолщения
        thickenInput = thickenFeats.createInput(bodiesToThicken, thicknessValue, False, adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
        
        # Отключаем выбор связанных поверхностей
        thickenInput.isChainSelection = False
        
        # Выполняем операцию утолщения
        try:
            thickenResult = thickenFeats.add(thickenInput)
        except Exception as e:
            ui.messageBox(f'Не удалось создать твердое тело:\n{str(e)}')
            # Удаляем смещенную поверхность
            if offsetBody.isValid:
                removeFeats = rootComp.features.removeFeatures
                removeFeats.add(offsetBody)
            return
        
        # Получаем созданное твердое тело
        if thickenResult.bodies.count == 0:
            ui.messageBox('Не удалось создать твердое тело из смещенной поверхности.')
            # Удаляем смещенную поверхность
            if offsetBody.isValid:
                removeFeats = rootComp.features.removeFeatures
                removeFeats.add(offsetBody)
            return
        
        thickenBody = thickenResult.bodies.item(0)
        
        # ЭТАП 7: РАЗРЕЗАНИЕ ЦЕЛЕВОЙ ТРУБЫ
        # Получаем функции разделения тела
        splitBodyFeats = rootComp.features.splitBodyFeatures
        
        # Создаем объект ввода для операции разделения тела
        splitBodyInput = splitBodyFeats.createInput(targetBody, thickenBody, False)
        
        # Выполняем операцию разделения тела
        ui.messageBox('Выполняем обрезку целевой трубы...')
        try:
            splitBodyResult = splitBodyFeats.add(splitBodyInput)
        except Exception as e:
            ui.messageBox(f'Не удалось разрезать целевую трубу:\n{str(e)}')
            # Удаляем созданные вспомогательные тела
            try:
                removeFeats = rootComp.features.removeFeatures
                if offsetBody.isValid:
                    removeFeats.add(offsetBody)
                if thickenBody.isValid:
                    removeFeats.add(thickenBody)
            except:
                pass
            return
        
        # ЭТАП 8: УДАЛЕНИЕ ОТРЕЗАННОЙ ЧАСТИ И ВСПОМОГАТЕЛЬНЫХ ТЕЛ
        # Находим все тела, которые были созданы в результате разделения
        
        # Получаем все тела в компоненте
        allBodies = rootComp.bRepBodies
        
        # Находим тела, которые появились после разделения
        # Это будут тела, отличные от исходных и вспомогательных тел
        newBodies = []
        originalBodies = [cuttingBody, targetBody, offsetBody, thickenBody]
        
        for i in range(allBodies.count):
            body = allBodies.item(i)
            isOriginal = False
            
            # Проверяем, не является ли это тело одним из исходных
            for originalBody in originalBodies:
                if body == originalBody:
                    isOriginal = True
                    break
            
            # Если это новое тело, добавляем его в список
            if not isOriginal:
                newBodies.append(body)
        
        # Находим меньшую часть по объему - это будет отрезанная часть
        if newBodies:
            smallestBody = None
            smallestVolume = float('inf')
            
            for body in newBodies:
                volume = body.volume
                if volume < smallestVolume:
                    smallestVolume = volume
                    smallestBody = body
            
            # Удаляем меньшую часть (отрезанную) и вспомогательные тела
            removeFeats = rootComp.features.removeFeatures
            
            if smallestBody and smallestBody.isValid:
                removeFeats.add(smallestBody)
                ui.messageBox('Отрезанная часть трубы успешно удалена.')
            
            # Удаляем вспомогательные тела
            if offsetBody.isValid:
                removeFeats.add(offsetBody)
            if thickenBody.isValid:
                removeFeats.add(thickenBody)
                
            # Обновляем targetBody - после разрезания это новое тело
            # Находим новое тело, которое осталось после удаления отрезанной части
            remainingBodies = []
            for i in range(rootComp.bRepBodies.count):
                body = rootComp.bRepBodies.item(i)
                if body != cuttingBody and body != smallestBody and body != offsetBody and body != thickenBody:
                    remainingBodies.append(body)
            
            if remainingBodies:
                # Находим тело с наибольшим объемом - это должна быть оставшаяся часть целевой трубы
                largestBody = None
                largestVolume = 0
                
                for body in remainingBodies:
                    volume = body.volume
                    if volume > largestVolume:
                        largestVolume = volume
                        largestBody = body
                
                # Обновляем targetBody
                if largestBody:
                    targetBody = largestBody
        else:
            ui.messageBox('Не найдено новых тел после операции разделения.')
            # Удаляем вспомогательные тела
            removeFeats = rootComp.features.removeFeatures
            if offsetBody.isValid:
                removeFeats.add(offsetBody)
            if thickenBody.isValid:
                removeFeats.add(thickenBody)
        
        # ЭТАП 9: ПОИСК И ВИЗУАЛИЗАЦИЯ ИНТЕРФЕРЕНЦИИ
        # Спрашиваем пользователя, хочет ли он увидеть анализ пересечений
        show_interference = ui.messageBox("Хотите ли вы увидеть пересечения после обработки?", 
                                         "Анализ пересечений", 
                                         adsk.core.MessageBoxButtonTypes.YesNoButtonType)
        
        if show_interference == adsk.core.DialogResults.DialogYes:
            # Проверяем, что дизайн типа DirectDesignType, необходимо для создания тел интерференции
            if design.designType != adsk.fusion.DesignTypes.DirectDesignType:
                ui.messageBox("Для визуализации интерференции требуется режим прямого моделирования. "
                              "Будет выполнен только анализ без создания тел.")
            
            # Создаем коллекцию для анализа интерференции со всеми телами
            entities = adsk.core.ObjectCollection.create()
            
            # Добавляем все твёрдые тела в компоненте в коллекцию для анализа
            allBodies = rootComp.bRepBodies
            for i in range(allBodies.count):
                body = allBodies.item(i)
                if body.isSolid:  # Только твердые тела (не поверхности)
                    entities.add(body)
            
            # Создаем объект ввода для анализа интерференции
            interferenceInput = design.createInterferenceInput(entities)
            
            # Устанавливаем параметры анализа интерференции
            interferenceInput.areCoincidentFacesIncluded = True
            
            # Вычисляем интерференции
            interferenceResults = design.analyzeInterference(interferenceInput)
            
            # Проверка, есть ли интерференции
            if interferenceResults.count > 0:
                # Если дизайн поддерживает создание тел интерференции
                if design.designType == adsk.fusion.DesignTypes.DirectDesignType:
                    # Для каждого результата интерференции
                    for i in range(interferenceResults.count):
                        interferenceResult = interferenceResults.item(i)
                        # Устанавливаем флаг, что нужно создать тела
                        interferenceResult.isCreateBody = True
                    
                    # Создаем тела интерференции
                    # Указываем True для аргумента allInterferenceBodies
                    interferenceResults.createBodies(True)
                
                ui.messageBox(f"Найдено {interferenceResults.count} пересечений между телами.")
            else:
                ui.messageBox("Пересечений между телами не найдено.")
        
        ui.messageBox('Операция успешно завершена! Труба обрезана для сварочного соединения.')
        
    except:
        # Если произошла ошибка, выводим ее
        if ui:
            ui.messageBox('Произошла ошибка:\n{}'.format(traceback.format_exc()))
            # Пытаемся откатить операцию при ошибке
            try:
                design.timeline.rollTo(startIndex)
            except:
                pass